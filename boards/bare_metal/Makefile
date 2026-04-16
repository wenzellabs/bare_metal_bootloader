# bare_metal USB bootloader - Makefile
# based on TinyFPGA BX bootloader build flow

PROJ     = bootloader
PIN_DEF  = pins.pcf
DEVICE   = up5k
PKG      = sg48

# Path to TinyFPGA bootloader common Verilog
COMMON   = ../../common

# All source files
SRC      = $(PROJ).v $(wildcard $(COMMON)/*.v)

USER_BITSTREAM ?= bootloader_copy.bin

# flash memory map

# 0x000_0000 - 0x000_009f =160b, multiboot header, slot table
# 0x000_00a0 - 0x001_ffff ~124kB, slot 0, bare_metal_bootloader
# 0x001_f000 - 0x001_ffff =4kB, bootmetadata
# 0x002_0000 - 0x003_ffff =128kB, slot 1, user bitstream, hackerstacker_demo by default
# 0x004_0000 - 0x005_ffff =128kB, slot 2, transputers anyone?
# 0x006_0000 - 0x007_ffff ~128kB, slot 3, hooray for transputers
# 0x008_0000 - 0x100_0000 user data, images, fonts, text, sounds, etc

# UP5K bitstream is always ~104 KB, so we need 2^17 = 128 KB alignment
ALIGN    = 17

all: $(PROJ).rpt multiboot.bin

$(PROJ).json: $(SRC)
	yosys -p 'synth_ice40 -top $(PROJ) -json $@' $(SRC)

$(PROJ).asc: $(PIN_DEF) $(PROJ).json
	nextpnr-ice40 --$(DEVICE) --package $(PKG) --pcf $(PIN_DEF) --json $(PROJ).json --asc $@

$(PROJ).bin: $(PROJ).asc
	icepack $< $@

# Metadata offset within slot 0 (must be within the 128KB slot, after bitstream)
META_ADDR = 0x1F000

# Build multiboot image:
#   -p0  = power-on image is slot 0 (bootloader)
#   -a17 = each image aligned at 128 KB boundary
# For now, slot 1 is a copy of the bootloader (placeholder).
# Replace bootloader_copy.bin with your user design when ready.
# After icemulti, inject minified bootmeta.json at META_ADDR.
multiboot.bin: $(PROJ).bin bootmeta.json
	cp $(PROJ).bin bootloader_copy.bin
	icemulti -v -o $@ -a$(ALIGN) -p0 bootloader.bin $(USER_BITSTREAM)
	python3 -c "\
import json; \
d = json.load(open('bootmeta.json')); \
blob = json.dumps(d, separators=(',',':')).encode(); \
assert len(blob) <= 4096, f'bootmeta too large: {len(blob)} bytes (max 4096)'; \
f = open('$@', 'r+b'); \
f.seek($(META_ADDR)); \
f.write(b'\\xff' * 4096); \
f.seek($(META_ADDR)); \
f.write(blob); \
f.close(); \
print(f'  Injected {len(blob)} bytes of bootmeta at $(META_ADDR)')"

$(PROJ).rpt: $(PROJ).asc
	icetime -d $(DEVICE) -mtr $@ $<

# erase SPI flash (required before 'make prog' on non-empty flash)
# leaving manually as it's time-consuming
erase:
	ch341prog -v -e

# Flash multiboot image via CH341A clip programmer
prog: multiboot.bin
	ch341prog -v -w multiboot.bin

# Flash multiboot image via iceprog (if available)
iceprog: multiboot.bin
	iceprog $<

# stage_two.bin: slot 0 with valid multiboot header, for OTA update.
# Written to address 0x0 by tinyprog during stage two of bootloader update.
# Uses icemulti with TWO images (so the header has a valid slot 1 entry),
# injects bootmeta, then truncates to 0x20000 (slot 0 boundary).
stage_two.bin: $(PROJ).bin bootmeta.json
	cp $(PROJ).bin bootloader_copy.bin
	icemulti -v -o $@ -a$(ALIGN) -p0 bootloader.bin bootloader_copy.bin
	python3 -c "\
import json; \
d = json.load(open('bootmeta.json')); \
blob = json.dumps(d, separators=(',',':')).encode(); \
assert len(blob) <= 4096, f'bootmeta too large: {len(blob)} bytes (max 4096)'; \
f = open('$@', 'r+b'); \
f.seek($(META_ADDR)); \
f.write(b'\\xff' * 4096); \
f.seek($(META_ADDR)); \
f.write(blob); \
f.close(); \
# truncate to slot 0 boundary - keeps header + bootloader + bootmeta \
data = open('$@','rb').read()[:0x20000]; \
# strip trailing 0xff, round up to 256B \
end = len(data); \
while end > 0 and data[end-1] == 0xff: end -= 1; \
end = (end + 255) & ~255; \
open('$@','wb').write(data[:end]); \
print(f'  stage_two.bin: {end} bytes (bootmeta at $(META_ADDR))')"

# User address in flash - must match icemulti -a$(ALIGN)
USER_ADDR = 0x20000

# Build a multiboot image with a real user design
# Usage: make multiboot-user USER_BIN=path/to/user.bin
multiboot-user: $(PROJ).bin
	icemulti -v -o multiboot.bin -a$(ALIGN) -p0 $(PROJ).bin $(USER_BIN)

# Just boot to user image (no programming)
boot:
	tinyprog --boot

clean:
	rm -f $(PROJ).json $(PROJ).asc $(PROJ).rpt $(PROJ).bin
	rm -f bootloader_copy.bin multiboot.bin stage_two.bin

# production: provision one board via CH341
# reads serial from next_serial.txt, generates a UUID, writes boardmeta
# to security register page 1, flashes multiboot.bin to address 0.
CH341PROG ?= ch341prog
SECREG_PAGE = 1

produce_one_board: multiboot.bin
	@echo "=== Producing board ==="
	@SERIAL=$$(cat next_serial.txt) && \
	UUID=$$(python3 -c "import uuid; print(uuid.uuid4())") && \
	echo "  Serial: $$SERIAL" && \
	echo "  UUID:   $$UUID" && \
	python3 -c "\
import json, sys; \
d = json.load(open('boardmeta.json')); \
d['boardmeta']['serial'] = int(sys.argv[1]); \
d['boardmeta']['uuid'] = sys.argv[2]; \
open('_boardmeta_prod.json','w').write(json.dumps(d, separators=(',',':')))" \
	    "$$SERIAL" "$$UUID" && \
	echo "  Minified: $$(wc -c < _boardmeta_prod.json) bytes (max 256)" && \
	test $$(wc -c < _boardmeta_prod.json) -le 256 || { echo "ERROR: boardmeta > 256 bytes!"; rm -f _boardmeta_prod.json; exit 1; } && \
	$(CH341PROG) -W $(SECREG_PAGE) _boardmeta_prod.json && \
	$(CH341PROG) -L $(SECREG_PAGE) && \
	$(CH341PROG) -w multiboot.bin && \
	echo $$(($$SERIAL + 1)) > next_serial.txt && \
	echo "=== Board $$SERIAL done. Next serial: $$(cat next_serial.txt) ===" && \
	rm -f _boardmeta_prod.json

.PHONY: all prog iceprog multiboot-user flash-user boot clean erase produce_one_board
