CC      = gcc
CFLAGS  = -O2 -Wall -Wextra \
          $(shell pkg-config --cflags gstreamer-1.0 gstreamer-app-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0)
LDFLAGS = -lpthread \
          $(shell pkg-config --libs   gstreamer-1.0 gstreamer-app-1.0 gstreamer-webrtc-1.0 gstreamer-sdp-1.0)

all: stream_server webrtc_server

stream_server: stream_server.c
	$(CC) $(CFLAGS) $< -o $@ \
	    $(shell pkg-config --libs gstreamer-1.0 gstreamer-app-1.0) -lpthread

webrtc_server: webrtc_server.c
	$(CC) $(CFLAGS) $< -o $@ $(LDFLAGS)

clean:
	rm -f stream_server webrtc_server
