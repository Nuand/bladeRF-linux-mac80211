CFLAGS=$(shell pkg-config --cflags libnl-genl-3.0 libbladeRF) -g3
LDLAGS=$(shell pkg-config --libs libnl-genl-3.0 libbladeRF) -lpthread
all: bladeRF-linux-mac80211

bladeRF-linux-mac80211: bladeRF-linux-mac80211.c
	gcc -o bladeRF-linux-mac80211 -g3 bladeRF-linux-mac80211.c $(CFLAGS) $(LDLAGS)

clean:
	rm bladeRF-linux-mac80211
