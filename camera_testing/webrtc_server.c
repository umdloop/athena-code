/*
 * webrtc_server.c
 * WebRTC camera server for Jetson radio interface (192.168.88.10).
 *
 * Each camera gets a WebRTC peer connection negotiated over a minimal
 * WebSocket signaling channel — no external dependencies beyond GStreamer.
 *
 * Protocol (JSON over WebSocket):
 *   client -> server:  { "type": "request",  "cameraId": 2 }
 *   server -> client:  { "type": "offer",    "sdp": "..." }
 *   client -> server:  { "type": "answer",   "sdp": "..." }
 *   client -> server:  { "type": "ice",      "candidate": "...", "sdpMLineIndex": 0 }
 *   server -> client:  { "type": "ice",      "candidate": "...", "sdpMLineIndex": 0 }
 *
 * Build:
 *   gcc webrtc_server.c -o webrtc_server \
 *       $(pkg-config --cflags --libs gstreamer-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0) \
 *       -lpthread
 *
 * Run:
 *   ./webrtc_server          # listens on 192.168.88.10:7001
 *   GST_CAMERA_PORT=7001 ./webrtc_server
 */

#include <arpa/inet.h>
#include <errno.h>
#include <gst/gst.h>
#include <gst/sdp/sdp.h>
#include <gst/webrtc/webrtc.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

/* ── SHA-1 (for WebSocket handshake) ─────────────────────────────────────── */
/* Minimal self-contained SHA-1 — no OpenSSL needed. */

typedef struct { uint32_t h[5]; uint8_t buf[64]; uint64_t bits; size_t used; } SHA1;

static void sha1_init(SHA1 *s) {
    s->h[0]=0x67452301; s->h[1]=0xEFCDAB89; s->h[2]=0x98BADCFE;
    s->h[3]=0x10325476; s->h[4]=0xC3D2E1F0;
    s->bits=0; s->used=0;
}
#define ROL32(v,n) (((v)<<(n))|((v)>>(32-(n))))
static void sha1_block(SHA1 *s) {
    uint32_t w[80], a,b,c,d,e,f,k,t;
    for(int i=0;i<16;i++) w[i]=((uint32_t)s->buf[i*4]<<24)|((uint32_t)s->buf[i*4+1]<<16)|((uint32_t)s->buf[i*4+2]<<8)|s->buf[i*4+3];
    for(int i=16;i<80;i++) w[i]=ROL32(w[i-3]^w[i-8]^w[i-14]^w[i-16],1);
    a=s->h[0];b=s->h[1];c=s->h[2];d=s->h[3];e=s->h[4];
    for(int i=0;i<80;i++){
        if(i<20){f=(b&c)|(~b&d);k=0x5A827999;}
        else if(i<40){f=b^c^d;k=0x6ED9EBA1;}
        else if(i<60){f=(b&c)|(b&d)|(c&d);k=0x8F1BBCDC;}
        else{f=b^c^d;k=0xCA62C1D6;}
        t=ROL32(a,5)+f+e+k+w[i]; e=d; d=c; c=ROL32(b,30); b=a; a=t;
    }
    s->h[0]+=a;s->h[1]+=b;s->h[2]+=c;s->h[3]+=d;s->h[4]+=e;
}
static void sha1_update(SHA1 *s, const uint8_t *data, size_t len) {
    for(size_t i=0;i<len;i++){
        s->buf[s->used++]=data[i]; s->bits+=8;
        if(s->used==64){sha1_block(s);s->used=0;}
    }
}
static void sha1_final(SHA1 *s, uint8_t out[20]) {
    s->buf[s->used++]=0x80;
    if(s->used>56){while(s->used<64)s->buf[s->used++]=0;sha1_block(s);s->used=0;}
    while(s->used<56)s->buf[s->used++]=0;
    for(int i=7;i>=0;i--){s->buf[56+(7-i)]=(uint8_t)(s->bits>>(i*8));}
    sha1_block(s);
    for(int i=0;i<5;i++){out[i*4]=(uint8_t)(s->h[i]>>24);out[i*4+1]=(uint8_t)(s->h[i]>>16);out[i*4+2]=(uint8_t)(s->h[i]>>8);out[i*4+3]=(uint8_t)s->h[i];}
}

/* ── Base64 encode ────────────────────────────────────────────────────────── */

static const char B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
static void base64_encode(const uint8_t *in, size_t len, char *out) {
    size_t i = 0, j = 0;
    while (i < len) {
        uint32_t v = 0;
        uint8_t a = i < len ? in[i++] : 0;
        uint8_t b = i < len ? in[i++] : 0;
        uint8_t c = i < len ? in[i++] : 0;
        v = ((uint32_t)a << 16) | ((uint32_t)b << 8) | c;
        out[j++] = B64[(v >> 18) & 63];
        out[j++] = B64[(v >> 12) & 63];
        out[j++] = (len % 3 == 1 && i >= len + 2) ? '=' : B64[(v >> 6) & 63];
        out[j++] = (len % 3 != 0 && i >= len + 1) ? '=' : B64[v & 63];
    }
    out[j] = '\0';
}

#define BIND_ADDR    "192.168.88.10"
#define DEFAULT_PORT 7001
#define BACKLOG      16
#define MAX_VIDEO_ID 99

/* ── WebSocket framing ────────────────────────────────────────────────────── */

static int ws_handshake(int fd) {
    char buf[2048] = {0};
    ssize_t n = recv(fd, buf, sizeof(buf)-1, 0);
    if (n <= 0) return -1;

    fprintf(stderr, "[ws] handshake request: %.80s\n", buf);

    /* must be a GET upgrade request */
    if (strncmp(buf, "GET ", 4) != 0) return -1;

    /* extract Sec-WebSocket-Key */
    char *key_start = strstr(buf, "Sec-WebSocket-Key:");
    if (!key_start) {
        fprintf(stderr, "[ws] no WebSocket key found — plain HTTP\n");
        return -1;
    }
    key_start += 18;
    while (*key_start == ' ') key_start++;
    char key[64] = {0};
    size_t ki = 0;
    while (ki < 63 && key_start[ki] && key_start[ki] != '\r' && key_start[ki] != '\n') {
        key[ki] = key_start[ki];
        ki++;
    }

    /* compute accept = base64(sha1(key + GUID)) */
    char combined[128];
    snprintf(combined, sizeof(combined), "%s258EAFA5-E914-47DA-95CA-C5AB0DC85B11", key);
    SHA1 sha; uint8_t digest[20]; char accept[32];
    sha1_init(&sha);
    sha1_update(&sha, (uint8_t *)combined, strlen(combined));
    sha1_final(&sha, digest);
    base64_encode(digest, 20, accept);

    char resp[512];
    int rlen = snprintf(resp, sizeof(resp),
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        "Sec-WebSocket-Accept: %s\r\n\r\n", accept);
    return send(fd, resp, (size_t)rlen, MSG_NOSIGNAL) > 0 ? 0 : -1;
}

/* Send a text frame. */
static int ws_send(int fd, const char *text) {
    size_t len = strlen(text);
    uint8_t hdr[10]; int hlen;
    hdr[0] = 0x81; /* FIN + text opcode */
    if (len < 126) { hdr[1] = (uint8_t)len; hlen = 2; }
    else if (len < 65536) {
        hdr[1]=126; hdr[2]=(uint8_t)(len>>8); hdr[3]=(uint8_t)len; hlen=4;
    } else {
        hdr[1]=127;
        for(int i=0;i<8;i++) hdr[2+i]=(uint8_t)(len>>((7-i)*8));
        hlen=10;
    }
    if (send(fd, hdr, (size_t)hlen, MSG_NOSIGNAL) < 0) return -1;
    return send(fd, text, len, MSG_NOSIGNAL) > 0 ? 0 : -1;
}

/* Read one WebSocket text frame into buf (caller provides buf+size).
   Returns length or -1 on error/close. */
/* Reliable recv: keeps reading until exactly `len` bytes received. */
static int recv_exact(int fd, uint8_t *buf, size_t len) {
    size_t got = 0;
    while (got < len) {
        ssize_t n = recv(fd, buf + got, len - got, 0);
        if (n <= 0) { if (n < 0 && errno == EINTR) continue; return -1; }
        got += (size_t)n;
    }
    return 0;
}

static ssize_t ws_recv(int fd, char *buf, size_t size) {
    uint8_t hdr[2];
    if (recv_exact(fd, hdr, 2) < 0) return -1;

    int opcode = hdr[0] & 0x0f;
    if (opcode == 0x8) return -1; /* close */

    bool masked = (hdr[1] & 0x80) != 0;
    uint64_t len = hdr[1] & 0x7f;

    if (len == 126) {
        uint8_t ext[2];
        if (recv_exact(fd, ext, 2) < 0) return -1;
        len = ((uint64_t)ext[0] << 8) | ext[1];
    } else if (len == 127) {
        uint8_t ext[8];
        if (recv_exact(fd, ext, 8) < 0) return -1;
        len = 0;
        for (int i = 0; i < 8; i++) len = (len << 8) | ext[i];
    }

    uint8_t mask[4] = {0};
    if (masked && recv_exact(fd, mask, 4) < 0) return -1;

    if (len >= size) {
        /* drain and discard oversized frame */
        uint8_t tmp[256];
        uint64_t rem = len;
        while (rem > 0) {
            size_t chunk = rem < sizeof(tmp) ? (size_t)rem : sizeof(tmp);
            ssize_t n = recv(fd, tmp, chunk, 0);
            if (n <= 0) return -1;
            rem -= (uint64_t)n;
        }
        return -1;
    }

    if (recv_exact(fd, (uint8_t *)buf, (size_t)len) < 0) return -1;
    if (masked) for (uint64_t i = 0; i < len; i++) buf[i] ^= mask[i % 4];
    buf[len] = '\0';
    return (ssize_t)len;
}

/* ── minimal JSON helpers ─────────────────────────────────────────────────── */

/* Extract string value for a key from flat JSON. Caller must g_free result. */
static char *json_get_string(const char *json, const char *key) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char *p = strstr(json, search);
    if (!p) return NULL;
    p += strlen(search);
    while (*p == ' ' || *p == ':' || *p == ' ') p++;
    if (*p != '"') return NULL;
    p++;
    const char *end = strchr(p, '"');
    if (!end) return NULL;
    return g_strndup(p, (gsize)(end - p));
}

static int json_get_int(const char *json, const char *key, int fallback) {
    char search[64];
    snprintf(search, sizeof(search), "\"%s\"", key);
    const char *p = strstr(json, search);
    if (!p) return fallback;
    p += strlen(search);
    while (*p == ' ' || *p == ':') p++;
    char *end; long v = strtol(p, &end, 10);
    return end == p ? fallback : (int)v;
}

/* ── WebRTC peer ──────────────────────────────────────────────────────────── */

typedef struct {
    int fd;           /* WebSocket fd */
    int camera_id;
    GstElement *pipe;
    GstElement *webrtc;
    pthread_mutex_t lock;
} Peer;

static void peer_send_json(Peer *peer, const char *json) {
    pthread_mutex_lock(&peer->lock);
    ws_send(peer->fd, json);
    pthread_mutex_unlock(&peer->lock);
}

static void on_ice_candidate(GstElement *webrtc G_GNUC_UNUSED,
                              guint mline, gchar *candidate, Peer *peer) {
    /* escape candidate string */
    GString *s = g_string_new(NULL);
    g_string_append_printf(s,
        "{\"type\":\"ice\",\"sdpMLineIndex\":%u,\"candidate\":\"%s\"}",
        mline, candidate);
    peer_send_json(peer, s->str);
    g_string_free(s, TRUE);
}

static void on_offer_created(GstPromise *promise, Peer *peer) {
    const GstStructure *reply = gst_promise_get_reply(promise);
    GstWebRTCSessionDescription *offer = NULL;
    gst_structure_get(reply, "offer", GST_TYPE_WEBRTC_SESSION_DESCRIPTION, &offer, NULL);
    gst_promise_unref(promise);

    /* set local description */
    GstPromise *p2 = gst_promise_new();
    g_signal_emit_by_name(peer->webrtc, "set-local-description", offer, p2);
    gst_promise_interrupt(p2);
    gst_promise_unref(p2);

    /* send offer to client */
    gchar *sdp_str = gst_sdp_message_as_text(offer->sdp);
    GString *msg = g_string_new(NULL);
    /* escape newlines in SDP for JSON */
    g_string_append(msg, "{\"type\":\"offer\",\"sdp\":\"");
    for (gchar *c = sdp_str; *c; c++) {
        if (*c == '\n')      g_string_append(msg, "\\n");
        else if (*c == '\r') g_string_append(msg, "\\r");
        else if (*c == '"')  g_string_append(msg, "\\\"");
        else if (*c == '\\') g_string_append(msg, "\\\\");
        else                 g_string_append_c(msg, *c);
    }
    g_string_append(msg, "\"}");
    peer_send_json(peer, msg->str);
    g_string_free(msg, TRUE);
    g_free(sdp_str);
    gst_webrtc_session_description_free(offer);
}

static void on_negotiation_needed(GstElement *webrtc, Peer *peer) {
    GstPromise *promise = gst_promise_new_with_change_func(
        (GstPromiseChangeFunc)on_offer_created, peer, NULL);
    g_signal_emit_by_name(webrtc, "create-offer", NULL, promise);
}

static char *build_webrtc_pipeline(int camera_id) {
    /* Try hardware encoder first (Jetson), fall back to software */
    int use_hw = system("gst-inspect-1.0 nvv4l2h264enc >/dev/null 2>&1") == 0;

    if (use_hw) {
        return g_strdup_printf(
            "v4l2src device=/dev/video%d io-mode=2 do-timestamp=true ! "
            "image/jpeg,width=640,height=480,framerate=30/1 ! "
            "jpegdec ! videoconvert ! "
            "nvv4l2h264enc bitrate=2000000 ! "
            "rtph264pay config-interval=-1 pt=96 ! "
            "webrtcbin name=webrtc bundle-policy=max-bundle",
            camera_id);
    } else {
        return g_strdup_printf(
            "v4l2src device=/dev/video%d io-mode=2 do-timestamp=true ! "
            "image/jpeg,width=640,height=480,framerate=30/1 ! "
            "jpegdec ! videoconvert ! "
            "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast ! "
            "rtph264pay config-interval=-1 pt=96 ! "
            "webrtcbin name=webrtc bundle-policy=max-bundle",
            camera_id);
    }
}

static Peer *peer_create(int fd, int camera_id) {
    Peer *peer = calloc(1, sizeof(Peer));
    peer->fd = fd;
    peer->camera_id = camera_id;
    pthread_mutex_init(&peer->lock, NULL);

    char *desc = build_webrtc_pipeline(camera_id);
    GError *err = NULL;
    peer->pipe = gst_parse_launch(desc, &err);
    g_free(desc);

    if (!peer->pipe) {
        fprintf(stderr, "[camera %d] pipeline error: %s\n",
                camera_id, err ? err->message : "?");
        if (err) g_error_free(err);
        free(peer);
        return NULL;
    }

    peer->webrtc = gst_bin_get_by_name(GST_BIN(peer->pipe), "webrtc");
    g_signal_connect(peer->webrtc, "on-negotiation-needed",
                     G_CALLBACK(on_negotiation_needed), peer);
    g_signal_connect(peer->webrtc, "on-ice-candidate",
                     G_CALLBACK(on_ice_candidate), peer);

    gst_element_set_state(peer->pipe, GST_STATE_PLAYING);
    fprintf(stderr, "[camera %d] WebRTC pipeline started\n", camera_id);
    return peer;
}

static void peer_destroy(Peer *peer) {
    if (!peer) return;
    if (peer->pipe) {
        gst_element_set_state(peer->pipe, GST_STATE_NULL);
        if (peer->webrtc) gst_object_unref(peer->webrtc);
        gst_object_unref(peer->pipe);
    }
    pthread_mutex_destroy(&peer->lock);
    free(peer);
}

static void peer_set_answer(Peer *peer, const char *sdp_str) {
    GstSDPMessage *sdp = NULL;
    if (gst_sdp_message_new_from_text(sdp_str, &sdp) != GST_SDP_OK) {
        fprintf(stderr, "[camera %d] bad SDP answer\n", peer->camera_id);
        return;
    }
    GstWebRTCSessionDescription *answer =
        gst_webrtc_session_description_new(GST_WEBRTC_SDP_TYPE_ANSWER, sdp);
    GstPromise *p = gst_promise_new();
    g_signal_emit_by_name(peer->webrtc, "set-remote-description", answer, p);
    gst_promise_interrupt(p);
    gst_promise_unref(p);
    gst_webrtc_session_description_free(answer);
}

static void peer_add_ice(Peer *peer, const char *candidate, int mline) {
    g_signal_emit_by_name(peer->webrtc, "add-ice-candidate", (guint)mline, candidate);
}

/* ── client handler ───────────────────────────────────────────────────────── */

static void handle_client(int fd) {
    if (ws_handshake(fd) < 0) {
        /* plain HTTP request — serve the test UI */
        static const char *body =
            "<!doctype html><html><head><meta charset=utf-8>"
            "<title>Cameras</title>"
            "<style>"
            "body{margin:0;background:#111;color:#fff;font-family:sans-serif}"
            "#grid{display:grid;grid-template-columns:repeat(5,1fr);gap:6px;padding:6px}"
            ".tile{position:relative;background:#222}"
            ".tile video{width:100%;display:block}"
            ".tile span{position:absolute;bottom:4px;left:4px;font-size:11px;"
            "background:rgba(0,0,0,.6);padding:2px 5px;border-radius:3px}"
            "</style></head><body>"
            "<div id=grid></div>"
            "<script>\n"
            "const WS='ws://'+window.location.hostname+':7001';\n"
            "const IDS=[2,4,7,8,10,13,14,16,20];\n"
            "function connect(id){\n"
            "  const tile=document.createElement('div');tile.className='tile';\n"
            "  const v=document.createElement('video');v.autoplay=true;v.muted=true;v.playsInline=true;\n"
            "  const lbl=document.createElement('span');lbl.textContent='cam '+id;\n"
            "  tile.append(v,lbl);document.getElementById('grid').appendChild(tile);\n"
            "  const pc=new RTCPeerConnection({iceServers:[]});\n"
            "  pc.ontrack=e=>{v.srcObject=e.streams[0];};\n"
            "  const ws=new WebSocket(WS);\n"
            "  ws.onopen=()=>ws.send(JSON.stringify({type:'request',cameraId:id}));\n"
            "  ws.onmessage=async e=>{\n"
            "    const m=JSON.parse(e.data);\n"
            "    if(m.type==='offer'){\n"
            "      await pc.setRemoteDescription({type:'offer',sdp:m.sdp});\n"
            "      const ans=await pc.createAnswer();\n"
            "      await pc.setLocalDescription(ans);\n"
            "      ws.send(JSON.stringify({type:'answer',sdp:ans.sdp}));\n"
            "    } else if(m.type==='ice'&&m.candidate){\n"
            "      pc.addIceCandidate({candidate:m.candidate,sdpMLineIndex:m.sdpMLineIndex});\n"
            "    } else if(m.type==='error'){\n"
            "      lbl.textContent='cam '+id+' ERR: '+m.message;\n"
            "    }\n"
            "  };\n"
            "  pc.onicecandidate=e=>{\n"
            "    if(e.candidate)ws.send(JSON.stringify({\n"
            "      type:'ice',candidate:e.candidate.candidate,\n"
            "      sdpMLineIndex:e.candidate.sdpMLineIndex}));\n"
            "  };\n"
            "  ws.onclose=()=>{pc.close();setTimeout(()=>connect(id),3000);};\n"
            "}\n"
            "IDS.forEach(connect);\n"
            "</script></body></html>\n";
        char hdr[256];
        snprintf(hdr, sizeof(hdr),
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\n"
            "Content-Length: %zu\r\nConnection: close\r\n\r\n", strlen(body));
        send(fd, hdr, strlen(hdr), MSG_NOSIGNAL);
        send(fd, body, strlen(body), MSG_NOSIGNAL);
        return;
    }

    Peer *peer = NULL;
    char msg[65536];
    fprintf(stderr, "[ws] client connected\n");

    while (true) {
        ssize_t n = ws_recv(fd, msg, sizeof(msg));
        if (n <= 0) { fprintf(stderr, "[ws] recv returned %zd errno=%d\n", n, errno); break; }

        char *type = json_get_string(msg, "type");
        if (!type) continue;

        if (strcmp(type, "request") == 0) {
            int cam_id = json_get_int(msg, "cameraId", -1);
            char devpath[32];
            snprintf(devpath, sizeof(devpath), "/dev/video%d", cam_id);
            if (cam_id < 0 || cam_id > MAX_VIDEO_ID || access(devpath, F_OK) != 0) {
                ws_send(fd, "{\"type\":\"error\",\"message\":\"camera not found\"}");
            } else {
                peer_destroy(peer);
                peer = peer_create(fd, cam_id);
                if (!peer)
                    ws_send(fd, "{\"type\":\"error\",\"message\":\"pipeline failed\"}");
            }

        } else if (strcmp(type, "answer") == 0 && peer) {
            char *sdp = json_get_string(msg, "sdp");
            if (sdp) { peer_set_answer(peer, sdp); g_free(sdp); }

        } else if (strcmp(type, "ice") == 0 && peer) {
            char *candidate = json_get_string(msg, "candidate");
            int mline = json_get_int(msg, "sdpMLineIndex", 0);
            if (candidate) { peer_add_ice(peer, candidate, mline); g_free(candidate); }
        }

        g_free(type);
    }

    peer_destroy(peer);
    fprintf(stderr, "[ws] client disconnected\n");
}

/* ── server loop ──────────────────────────────────────────────────────────── */

static void *client_thread(void *arg) {
    int fd = *(int *)arg; free(arg);
    handle_client(fd);
    close(fd);
    return NULL;
}

static int get_port(void) {
    const char *e = getenv("GST_WEBRTC_PORT");
    if (!e || !*e) return DEFAULT_PORT;
    char *end; long v = strtol(e, &end, 10);
    return (*end || v < 1 || v > 65535) ? DEFAULT_PORT : (int)v;
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
    if (inet_pton(AF_INET, "0.0.0.0", &addr.sin_addr) != 1) {
        fprintf(stderr, "invalid bind address\n");
        return 1;
    }

    if (bind(sfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); return 1; }
    if (listen(sfd, BACKLOG) < 0) { perror("listen"); return 1; }

    fprintf(stderr, "webrtc-camera-server on ws://%s:%d\n", BIND_ADDR, port);
    fprintf(stderr, "  send: {\"type\":\"request\",\"cameraId\":<id>}\n");

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
