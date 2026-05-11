$(call inherit-product, vendor/honda/hardware/interfaces/automotive/sepolicy.mk)
include device/honda/products/emulator_common/common/sepolicy/hardware/interfaces/automotive/sepolicy.mk
include device/honda/products/emulator_common/common/sepolicy/hardware/implements/automotive/automotivesepolicy.mk
include device/honda/products/emulator_common/common/sepolicy/misc/sepolicy.mk
#lyfecycle specific selinux policy
BOARD_VENDOR_SEPOLICY_DIRS += vendor/honda/lifecycle/sepolicy/vendor
