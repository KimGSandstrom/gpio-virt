# GPIO virtualization

The GPIO (General Purpose Input Output) virtualisation allows the virtual 
machines (VMs) to access GPIO resources. (such as GPIO pins and dedicated 
IO to devices)

The next diagram shows how drivers on host and guest communicate via proxies.
A memory range used by the proxy host driver and the proxy guest driver is 
mapped to each other by Qemu.

The memory is set up by **tegra186_gpio_probe** and published to Qemu and the
guest VM's GPIO drivers. Thus the guest VM's driver does not set up IO memory 
and the host's GPIO driver does not use it to manipulate GPIO.


                                            VM
                                   +------------------+
                                   |   GPIO client    |
                                   +------------------+
                                             | Reset/clocks
                                             v
                                   +------------------+
                                   | GPIO guest proxy |
                                   +------------------+
                                             |
                                  -----------|-----------
                                  VMM/Qemu   v
                                   +------------------+
                                   |   GPIO VMM guest |
                                   +------------------+
                                             |
                                  -----------|-----------
    Host                          Host       v
     +------------------+          +------------------+
     |   GPIO actor ?   |          | GPIO host proxy  |
     +------------------+          +------------------+
               | Reset/clocks                |
               v                             v
       +--------------+            +------------------+
       | GPIO driver  |            |   GPIO driver    |
       +--------------+            +------------------+


## General design assumptions

### GPIO host proxy

### GPIO VMM guest

### GPIO guest proxy

### GPIO driver

## Installation steps

1. Create a development environment with Ubuntu 20.04 on your Nvidia Orin.

2. Download the Nvidia L4T Driver Package (BSP) version 35.3.1 from (also
   tested with version 35.2.1):

		https://developer.nvidia.com/embedded/jetson-linux-r3531

3. Extract the Nvidia L4T Driver Package (BSP):

		tar -xvf Jetson_Linux_R35.3.1_aarch64.tbz2

4. Sync the source code to the tag *jetson_35.3.1*

		cd Linux_for_Tegra
		./source_sync.sh -t jetson_35.3.1

5. Clone this repository to Linux_for_Tegra/sources/kernel

		cd sources/kernel
		git clone https://github.com/jpruiz/gpio-virt.git

6. Apply patches from the repo with:

	TODO

7. TODO
    NOTE that the gpio-virt kernel overlay is added by the TODO patch. The line
   "gpio-virt" has been added to the files.

		kernel-5.10/kernel-int-overlays.txt
		kernel-5.10/kernel-overlays.txt

8. Check that the following configuration lines were added by the 'TODO.patch' to
   kernel-5.10/arch/arm64/configs/defconfig

		TODO CONFIG_VFIO=y
		TODO CONFIG_KVM_VFIO=y
		TODO CONFIG_VFIO_PLATFORM=y
		TODO CONFIG_TEGRA_GPIO_GUEST_PROXY=y
		TODO CONFIG_TEGRA_GPIO_HOST_PROXY=y

9. Compile the Linux kernel and the gpio-virt kernel overlay with the next commands

		cd Linux_for_Tegra/sources/kernel
		make -C kernel-5.10/ ARCH=arm64 O=../kernel_out -j12 defconfig
		make -C kernel-5.10/ ARCH=arm64 O=../kernel_out -j12 Image

    You will find the compiled kernel image in:

		kernel_out/arch/arm64/boot/Image

    TODO !!!
    **IMPORTANT NOTE:** use this same image for both host and kernel.


## GPIO passthrough instructions

This instructions describes the modifications on the device tree to passthrough
TODO
is BPMP dependent ?

1. For the host, modify the gpio node in Linux_for_Tegra/sources/hardware/
   nvidia/soc/t23x/

		kernel-dts/tegra234-soc/tegra234-soc-uart.dtsi

		TODO: tegra234-soc-fpga.dtsi

   with this content:

		tegra_main_gpio: gpio@2200000 {
			TODO
		};

		tegra_aon_gpio: gpio@c2f0000 {
			TODO
		};

    Alternatively you can apply the changes from a patch

		pushd path-to/Linux_for_Tegra/sources/hardware/nvidia/soc/t23x/
		patch -p1 < path-to/0006-gpio-host-uarta-dts.patch
		popd

    This places the UARTA alone in the IOMMU group TEGRA_SID_NISO1_SMMU_TEST.
    Also, this configuration disables the default nvidia,tegra194-hsuart driver
    by replacing it with a dummy driver.

4. Compile the device tree with the command:

 		cd Linux_for_Tegra/sources/kernel
 		make -C kernel-5.10/ ARCH=arm64 O=../kernel_out -j12 dtbs

	You will find the compiled device tree for Nvidia Jetson Orin AGX host on:

		kernel_out/arch/arm64/boot/dts/nvidia/tegra234-p3701-0000-p3737-0000.dtb

5. Copy kernel Image and tegra234-p3701-0000-p3737-0000.dtb files to the appropriate
   locations defined in

		/boot/extlinux/extlinux.conf

6. Start setting up the guest. Extract and edit guests Device Tree
	Dump the guests Device Tree from Qemu:

		qemu-system-aarch64 -machine virt,accel=kvm,dumpdtb=virt.dtb -cpu host

	Extract the dts code from the dtb file and copy it for editing:

		dtc -Idtb -Odts virt.dtb -o virt-qemu-8.1.0.dts
		cp virt-qemu-8.1.2.dts gpio-qemu-8.1.2.dts

	Now you can edit the guest Device Tree in 'gpio-qemu-8.1.0.dts'.

8. Edit the Qemu guest's device tree.

		nvim uarta-qemu-8.1.0.dts

	Add ... TODO

		node: so-and-so {
			TODO
		};

10. Compile the amended guest Device Tree

		dtc -Idts -Odtb gpio-qemu-8.1.2.dts -o gpio-qemu-8.1.2.dtb

11. Also, you will need to allow unsafe interrupts
	Either type as sudo

		echo 1 > /sys/module/vfio_iommu_type1/parameters/allow_unsafe_interrupts

	or add this kernel boot parameter in /boot/extlinux/extlinux.conf

		vfio_iommu_type1.allow_unsafe_interrupts=1

	After reboot, you can check the status with

		cat /sys/module/vfio_iommu_type1/parameters/allow_unsafe_interrupts

	or with

		modinfo vfio_iommu_type1

	You need to bind gpio to vfio-platform

		TODO
		echo vfio-platform > /sys/bus/platform/devices/2200000.gpio/driver_override
		echo 2200000.gpio > /sys/bus/platform/drivers/vfio-platform/bind

		echo vfio-platform > /sys/bus/platform/devices/c2f0000.gpio/driver_override
		echo c2f0000.gpio > /sys/bus/platform/drivers/vfio-platform/bind

	You can check if binding is successful with:

		ls -l /sys/bus/platform/drivers/vfio-platform/2200000.gpio
		ls -l /sys/bus/platform/drivers/vfio-platform/c2f0000.gpio

	A symbolic link should be present.

12. Finally you can run your VM. Use the following environment variables and Qemu command. 
    Qemu monitor will be on pty, VM console will be in the startup terminal:

		TODO

		rootfs="tegra_rootfs.qcow2"
		kernel=Image
		dtb=uarta-qemu-8.1.0.dtb
		sudo -E qemu-system-aarch64 \
		    -nographic \
		    -machine virt,accel=kvm \
		    -cpu host \
		    -m 4G \
		    -no-reboot \
		    -kernel ${kernel} \
		    -dtb ${dtb} \
		    -append "rootwait root=/dev/vda console=ttyAMA0" \
		    -device vfio-platform,host=3100000.serial \
		    -drive file=${rootfs},if=virtio,format=qcow2 \
		    -net user,hostfwd=tcp::2222-:22 -net nic \
		    -chardev pty,id=mon0 \
		    -mon chardev=mon0,mode=readline

13. To test the GPIO you can connect a logic analysator or some other detector
    to default GPIO pins on the 40-pin external connector

	On gpiochip0/tegra234-gpio/2200000.gpio
		Id	Name	Pin	Designation
		GPIO17	PP.04	Pin 22	GPIO       
		GPIO27	PN.01	Pin 15	GPIO/PWM
		GPIO35	PH.00	Pin 18	GPIO/PWM

	On gpiochip1/tegra234-gpio-aon/c2f0000.gpio
		Id	Name	Pin	Designation
		GPIO08	PBB.01	Pin 16	GPIO/AO DMIC
		GPIO09	PBB.00	Pin 32	GPIO/AO DMIC

	For example, we can test pins 15 and 16 with a logic analysator

        from the VM:
		testset="PN.01 PBB.01"
		for pin in ${testset}
		# export pins and set direction
		do
			echo $pin > /sys/class/gpio/export
			echo "out" > /sys/class/gpio/$pin/direction
		done
		# at this point exported pins should be visible as 
		# symbolic links in /sys/class/gpio/
		# toggle output
		for pin in ${testset}
		do
			echo 1 > /sys/class/gpio/$pin/value
			sleep 0.1
			echo 0 > /sys/class/gpio/$pin/value
		done
		# unexport pins
		for pin in ${testset}
		do
			echo $pin > /sys/class/gpio/unexport"
		done

	Note several pins can be in the exported state at the same time
