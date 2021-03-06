
ifeq ($(PROCESSOR),x86)
	include ../../gst_plugin_framework/Makefile-x86.conf
	override CFLAGS += -std=c99 -D_GNU_SOURCE -DGST_DISABLE_DEPRECATED
	override CFLAGS += -DDSP_API=$(DSP_API) -DSN_API=$(SN_API)
	override CFLAGS += -I../../gst_plugin_framework
else
	include ../../gst_plugin_framework/Makefile-ARM.conf
	override CFLAGS += -std=c99 -D_GNU_SOURCE -DGST_DISABLE_DEPRECATED -mcpu=cortex-a8 -mfpu=neon -mfloat-abi=softfp -ftree-vectorize
	override CFLAGS += -DDSP_API=$(DSP_API) -DSN_API=$(SN_API)
	override CFLAGS += -I../../gst_plugin_framework
endif
override CFLAGS += -lm -fno-builtin-cos -fno-builtin-sin -fno-builtin-acos -fno-builtin-asin -fno-builtin-atan

all:

D = $(DESTDIR)


gst_plugin := libMAVLab.so

$(gst_plugin):  gstardrone.o ardrone_pprz.o optic_flow.o trig.o ../../gst_plugin_framework/socket.o
$(gst_plugin): override CFLAGS += $(GST_CFLAGS) \
	-D VERSION='"$(version)"' -D DSPDIR='"$(dspdir)"'
$(gst_plugin): override LIBS += $(GST_LIBS)

targets += $(gst_plugin)

../../ardrone2_gstreamer/opt/arm/gst/include/gstreamer-0.10/gst/gst.h:
	make -C ../../ardrone2_gstreamer

upload:
	sb2 make -C ./ all && ../../ardrone2_gstreamer/ardrone2.py upload_gst_module $(gst_plugin)

doc: $(gst_plugin)
	$(MAKE) -C doc

doc-install: doc
	$(MAKE) -C doc install

all: $(targets) $(bins)

# pretty print
ifndef V
QUIET_CC    = @echo '   CC         '$@;
QUIET_LINK  = @echo '   LINK       '$@;
QUIET_CLEAN = @echo '   CLEAN      '$@;
endif

.PHONY: doc doc-install

%.so: override CFLAGS += -fPIC

install: $(targets) $(bins)
	install -m 755 -D libMAVLab.so $(D)$(prefix)/lib/gstreamer-0.10/libMAVLab.so

%.o:: %.c
	$(QUIET_CC)$(CC) $(CFLAGS) -MMD -MP -o $@ -c $<

$(bins):
	$(QUIET_LINK)$(CC) $(LDFLAGS) $^ $(LIBS) -o $@

%.so::
	$(QUIET_LINK)$(CC) $(LDFLAGS) -shared $^ $(LIBS) -o $@

%.a::
	$(QUIET_LINK)$(AR) rcs $@ $^

clean:
	echo $(PROCESSOR)
	$(QUIET_CLEAN)$(RM) -v $(targets) $(bins) *.o *.d tidsp/*.d tidsp/*.o ../../gst_plugin_framework/*.o ../../gst_plugin_framework/*.d

dist: base := gst-dsp-$(version)
dist:
	git archive --format=tar --prefix=$(base)/ HEAD > /tmp/$(base).tar
	mkdir -p $(base)
	echo $(version) > $(base)/.version
	chmod 664 $(base)/.version
	tar --append -f /tmp/$(base).tar --owner root --group root $(base)/.version
	rm -r $(base)
	gzip /tmp/$(base).tar

-include *.d tidsp/*.d
