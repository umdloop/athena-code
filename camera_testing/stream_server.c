/*
 * stream_server.c
 * MJPEG-over-HTTP camera server bound to 192.168.88.10 (Jetson radio).
 *
 * Routes:
 *   GET /health        - liveness check
 *   GET /cameras       - list /dev/video* devices
 *   GET /camera/<id>   - MJPEG stream from /dev/video<id>  (0-10)
 *
 * Build:
 *   gcc stream_server.c -o stream_server \
 *       $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-app-1.0) -lpthread
 */

#include <arpa/inet.h>
#include <errno.h>
#include <glob.h>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define BIND_ADDR       "192.168.88.10"
#define DEFAULT_PORT    7000
#define LISTEN_BACKLOG  16
#define MAX_VIDEO_ID    99
#define REQ_BUF         16384
#define HDR_BUF         512
#define DEFAULT_WIDTH   640
#define DEFAULT_HEIGHT  480
#define DEFAULT_FPS     30
#define DEFAULT_QUALITY 75

/* ── helpers ──────────────────────────────────────────────────────────────── */

static int write_all(int fd, const void *data, size_t len) {
    const unsigned char *p = data;
    while (len > 0) {
        ssize_t n = send(fd, p, len, MSG_NOSIGNAL);
        if (n < 0) { if (errno == EINTR) continue; return -1; }
        if (n == 0) return -1;
        p += n; len -= (size_t)n;
    }
    return 0;
}

static void send_response(int fd, int code, const char *status,
                          const char *ctype, const char *body) {
    char hdr[HDR_BUF];
    int n = snprintf(hdr, sizeof(hdr),
        "HTTP/1.1 %d %s\r\n"
        "Content-Type: %s\r\n"
        "Content-Length: %zu\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Connection: close\r\n\r\n",
        code, status, ctype, strlen(body));
    if (n > 0) { write_all(fd, hdr, (size_t)n); write_all(fd, body, strlen(body)); }
}

static int get_port(void) {
    const char *e = getenv("GST_CAMERA_PORT");
    if (!e || !*e) return DEFAULT_PORT;
    char *end; long v = strtol(e, &end, 10);
    return (*end || v < 1 || v > 65535) ? DEFAULT_PORT : (int)v;
}

static int env_int(const char *name, int def, int lo, int hi) {
    const char *e = getenv(name);
    if (!e || !*e) return def;
    char *end; long v = strtol(e, &end, 10);
    return (*end || v < lo || v > hi) ? def : (int)v;
}

/* ── GStreamer ─────────────────────────────────────────────────────────────── */

static char *gst_err_string(GstMessage *msg) {
    GError *err = NULL;
    gchar *dbg = NULL;
    gst_message_parse_error(msg, &err, &dbg);
    char *r = g_strdup_printf("%s%s%s",
        err ? err->message : "unknown",
        dbg ? " - " : "", dbg ? dbg : "");
    if (err) g_error_free(err);
    g_free(dbg);
    return r;
}

/* Returns NULL on success, caller-owned error string on failure. */
static char *open_pipeline(int camera_id,
                            GstElement **pipe_out, GstAppSink **sink_out) {
    int w = env_int("GST_CAMERA_WIDTH",   DEFAULT_WIDTH,   160, 3840);
    int h = env_int("GST_CAMERA_HEIGHT",  DEFAULT_HEIGHT,  120, 2160);
    int f = env_int("GST_CAMERA_FPS",     DEFAULT_FPS,     1,   120);
    int q = env_int("GST_CAMERA_QUALITY", DEFAULT_QUALITY, 1,   100);

    /* Use native MJPEG from camera (no re-encoding needed) */
    char *desc = g_strdup_printf(
        "v4l2src device=/dev/video%d io-mode=2 do-timestamp=true ! "
        "image/jpeg,width=%d,height=%d,framerate=%d/1 ! "
        "queue max-size-buffers=1 leaky=downstream ! "
        "jpegparse ! "
        "appsink name=sink emit-signals=false sync=false drop=true max-buffers=1",
        camera_id, w, h, f);
    (void)q; /* quality unused in passthrough mode */

    GError *err = NULL;
    GstElement *pipe = gst_parse_launch(desc, &err);
    g_free(desc);

    if (!pipe) {
        char *m = g_strdup_printf("pipeline parse failed: %s",
                                  err ? err->message : "?");
        if (err) g_error_free(err);
        return m;
    }

    GstElement *se = gst_bin_get_by_name(GST_BIN(pipe), "sink");
    if (!se || !GST_IS_APP_SINK(se)) {
        if (se) gst_object_unref(se);
        gst_object_unref(pipe);
        return g_strdup("appsink 'sink' not found");
    }

    if (gst_element_set_state(pipe, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) {
        GstBus *bus = gst_element_get_bus(pipe);
        GstMessage *msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
        char *detail = msg ? gst_err_string(msg) : g_strdup("failed to reach PLAYING");
        if (msg) gst_message_unref(msg);
        gst_object_unref(bus);
        gst_element_set_state(pipe, GST_STATE_NULL);
        gst_object_unref(se);
        gst_object_unref(pipe);
        return detail;
    }

    GstState state = GST_STATE_NULL;
    gst_element_get_state(pipe, &state, NULL, 3 * GST_SECOND);
    if (state != GST_STATE_PLAYING) {
        char *detail = g_strdup_printf("pipeline stuck in %s",
                                       gst_element_state_get_name(state));
        gst_element_set_state(pipe, GST_STATE_NULL);
        gst_object_unref(se);
        gst_object_unref(pipe);
        return detail;
    }

    *pipe_out = pipe;
    *sink_out = GST_APP_SINK(se);
    return NULL;
}

/* ── request handling ─────────────────────────────────────────────────────── */

static ssize_t read_request(int fd, char *buf, size_t size) {
    size_t used = 0;
    while (used + 1 < size) {
        ssize_t n = recv(fd, buf + used, size - used - 1, 0);
        if (n < 0) { if (errno == EINTR) continue; return -1; }
        if (n == 0) break;
        used += (size_t)n;
        buf[used] = '\0';
        if (strstr(buf, "\r\n\r\n")) break;
    }
    buf[used] = '\0';
    return (ssize_t)used;
}

static void stream_camera(int fd, int camera_id) {
    GstElement *pipe = NULL;
    GstAppSink *sink = NULL;
    char *err = open_pipeline(camera_id, &pipe, &sink);

    if (err) {
        char body[512];
        snprintf(body, sizeof(body), "{\"ok\":false,\"error\":\"%s\"}", err);
        send_response(fd, 503, "Service Unavailable", "application/json", body);
        fprintf(stderr, "[camera %d] %s\n", camera_id, err);
        g_free(err);
        return;
    }

    static const char *stream_hdr =
        "HTTP/1.1 200 OK\r\n"
        "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Cache-Control: no-cache\r\n"
        "X-Accel-Buffering: no\r\n"
        "Connection: close\r\n\r\n";

    if (write_all(fd, stream_hdr, strlen(stream_hdr)) < 0) goto cleanup;

    fprintf(stderr, "[camera %d] streaming started\n", camera_id);

    while (true) {
        GstSample *sample = gst_app_sink_try_pull_sample(sink, GST_SECOND);
        if (!sample) {
            GstBus *bus = gst_element_get_bus(pipe);
            GstMessage *msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
            if (msg) {
                char *e = gst_err_string(msg);
                fprintf(stderr, "[camera %d] %s\n", camera_id, e);
                g_free(e);
                gst_message_unref(msg);
            }
            gst_object_unref(bus);
            break;
        }

        GstBuffer *buf = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (!buf || !gst_buffer_map(buf, &map, GST_MAP_READ)) {
            gst_sample_unref(sample);
            break;
        }

        char fhdr[HDR_BUF];
        int fhdr_len = snprintf(fhdr, sizeof(fhdr),
            "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %zu\r\n\r\n",
            map.size);

        bool ok = fhdr_len > 0
            && write_all(fd, fhdr, (size_t)fhdr_len) == 0
            && write_all(fd, map.data, map.size) == 0
            && write_all(fd, "\r\n", 2) == 0;

        gst_buffer_unmap(buf, &map);
        gst_sample_unref(sample);
        if (!ok) break;
    }

    fprintf(stderr, "[camera %d] streaming stopped\n", camera_id);

cleanup:
    gst_element_set_state(pipe, GST_STATE_NULL);
    gst_object_unref(sink);
    gst_object_unref(pipe);
}

/* Returns true if the device supports MJPEG capture (i.e. it's a real camera). */
static bool device_supports_mjpeg(const char *devpath) {
    char cmd[128];
    snprintf(cmd, sizeof(cmd),
             "v4l2-ctl --device=%s --list-formats 2>/dev/null | grep -qc MJPG", devpath);
    return system(cmd) == 0;
}

static void handle_cameras_list(int fd) {
    GString *json = g_string_new("{\"ok\":true,\"cameras\":[");
    glob_t g;
    memset(&g, 0, sizeof(g));
    bool first = true;
    if (glob("/dev/video*", 0, NULL, &g) == 0) {
        for (size_t i = 0; i < g.gl_pathc; i++) {
            const char *dev = g.gl_pathv[i];
            /* extract numeric id */
            const char *numstart = dev + strlen("/dev/video");
            char *end = NULL;
            long id = strtol(numstart, &end, 10);
            if (*end != '\0') continue;
            if (!device_supports_mjpeg(dev)) continue;
            if (!first) g_string_append_c(json, ',');
            first = false;
            g_string_append_printf(json, "{\"id\":%ld,\"device\":\"%s\"}", id, dev);
        }
    }
    globfree(&g);
    g_string_append(json, "]}");
    char *body = g_string_free(json, FALSE);
    send_response(fd, 200, "OK", "application/json", body);
    g_free(body);
}

static void handle_client(int fd) {
    char req[REQ_BUF];
    if (read_request(fd, req, sizeof(req)) <= 0) return;

    char method[8] = {0}, path[128] = {0};
    if (sscanf(req, "%7s %127s", method, path) != 2) {
        send_response(fd, 400, "Bad Request", "application/json",
                      "{\"ok\":false,\"error\":\"bad request\"}");
        return;
    }

    if (strcmp(method, "GET") == 0 && strcmp(path, "/health") == 0) {
        send_response(fd, 200, "OK", "application/json",
                      "{\"ok\":true,\"service\":\"camera-streamer\"}");

    } else if (strcmp(method, "GET") == 0 && strcmp(path, "/cameras") == 0) {
        handle_cameras_list(fd);

    } else if (strcmp(method, "GET") == 0 && strncmp(path, "/camera/", 8) == 0) {
        char *end = NULL;
        long id = strtol(path + 8, &end, 10);
        char devpath[32];
        snprintf(devpath, sizeof(devpath), "/dev/video%ld", id);
        if (*end != '\0' || id < 0 || id > MAX_VIDEO_ID || access(devpath, F_OK) != 0) {
            send_response(fd, 404, "Not Found", "application/json",
                          "{\"ok\":false,\"error\":\"camera device not found\"}");
        } else {
            stream_camera(fd, (int)id);
        }

    } else {
        send_response(fd, 404, "Not Found", "application/json",
                      "{\"ok\":false,\"error\":\"not found\"}");
    }
}

/* ── server loop ──────────────────────────────────────────────────────────── */

static void *client_thread(void *arg) {
    int fd = *(int *)arg;
    free(arg);
    handle_client(fd);
    close(fd);
    return NULL;
}

int main(int argc, char **argv) {
    gst_init(&argc, &argv);
    signal(SIGPIPE, SIG_IGN);

    int port = get_port();

    int sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) { perror("socket"); return 1; }

    int yes = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    struct sockaddr_in addr = {0};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons((uint16_t)port);
    if (inet_pton(AF_INET, BIND_ADDR, &addr.sin_addr) != 1) {
        fprintf(stderr, "invalid bind address: %s\n", BIND_ADDR);
        return 1;
    }

    if (bind(sfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }
    if (listen(sfd, LISTEN_BACKLOG) < 0) { perror("listen"); return 1; }

    fprintf(stderr, "camera-streamer on http://%s:%d\n", BIND_ADDR, port);
    fprintf(stderr, "  GET /health\n");
    fprintf(stderr, "  GET /cameras\n");
    fprintf(stderr, "  GET /camera/<0-%d>\n", MAX_VIDEO_ID);

    while (true) {
        int cfd = accept(sfd, NULL, NULL);
        if (cfd < 0) { if (errno == EINTR) continue; perror("accept"); break; }

        setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));

        int *arg = malloc(sizeof(int));
        if (!arg) { close(cfd); continue; }
        *arg = cfd;

        pthread_t t;
        if (pthread_create(&t, NULL, client_thread, arg) != 0) {
            close(cfd); free(arg);
        } else {
            pthread_detach(t);
        }
    }

    close(sfd);
    return 0;
}
