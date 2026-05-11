TARGET_ARCH ?= x86_64

# Cuttlefish common build target
include device/honda/products/emulator_common/cuttlefish/device.mk

# Device specific overlay
# No Device overlays

# call PRODUCT_PACKAGES mk files.
-include device/honda/products/emu_cf_x86_64_honda_g_car/products/*.mk

# runtime resource overlay
ifneq ($(RRO_PRODUCT_PACKAGES),)
PRODUCT_PACKAGES += $(RRO_PRODUCT_PACKAGES)
endif

#--------lifecycle controller --------
PRODUCT_PACKAGES += \
           Lifecyclecontroller
           
ifneq ($(RRO_PRODUCT_PACKAGES_VENDOR),)
PRODUCT_PACKAGES += $(RRO_PRODUCT_PACKAGES_VENDOR)
endif

PRODUCT_NAME := emu_cf_x86_64_honda_g_car
PRODUCT_DEVICE := $(PRODUCT_NAME)
PRODUCT_BRAND := Honda
PRODUCT_MODEL := IVI-SYSTEM
PRODUCT_MANUFACTURER := $(PRODUCT_BRAND)

BUILD_HOST := $(PRODUCT_BRAND)
BUILD_HOSTNAME := $(PRODUCT_BRAND)
BUILD_PRODUCT := $(PRODUCT_NAME)
BUILD_USERNAME := $(PRODUCT_BRAND)
