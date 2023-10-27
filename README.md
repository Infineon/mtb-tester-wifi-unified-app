#  Unified Test Application (Wifi-Unified-App) for FreeRTOS

The Wifi-Unified-App combines these applications as wifi-bt-tester, wifi-mfg-tester, wifi-cert-tester and wifi-wlan-lowpower application, and it can execute the variant commands via two serial ports.
- wifi-bt-tester: Refer to https://github.com/Infineon/mtb-wifi-bluetooth-tester#wi-fi-bluetooth-tester
- wifi-mfg-tester: Refer to https://github.com/Infineon/mtb-anycloud-wifi-mfg-tester#psoc-6-mcu-wlan-manufacturing-test-application-wifi-mfg-tester-for-freertos
- wifi-cert-tester: Refer to https://github.com/Infineon/mtb-anycloud-wifi-cert-tester#wi-fi-cert-tester-tool-for-modustoolbox-sdk
- wifi-wlan-lowpower: Refer to https://github.com/Infineon/mtb-example-wifi-wlan-lowpower#wlan-low-power


## Requirements
- [ModusToolbox&trade; software](https://www.infineon.com/modustoolbox) v3.0 or later (tested with v3.0 and v3.1)
- Board support package (BSP) minimum required version: 4.0.0
- Programming Language: C
- Supported Toolchains: Arm® GCC
- Associated parts: All [PSoC&trade; 6 MCU](https://www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/psoc-6-32-bit-arm-cortex-m4-mcu/) parts with SDIO, [AIROC™ CYW4373 Wi-Fi & Bluetooth® combo chip](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/wi-fi-5-802.11ac/cyw4373/), [AIROC™ CYW43022 Wi-Fi & Bluetooth® combo chip](https://www.infineon.com/cms/en/product/wireless-connectivity/airoc-wi-fi-plus-bluetooth-combos/wi-fi-5-802.11ac/cyw43022/)

## Dependent assets
- [Wi-Fi connection manager](https://github.com/Infineon/wifi-connection-manager)
- [Command console](https://github.com/Infineon/command-console)
- [Wi-Fi Cert library](https://github.com/Infineon/wifi-cert)
- [Enterprise security](https://github.com/Infineon/enterprise-security)

## Supported toolchains (make variable 'TOOLCHAIN')
- GNU Arm® embedded compiler v10.3.1 (`GCC_ARM`) - Default value of `TOOLCHAIN`

## Supported kits (make variable 'TARGET')
- [PSoC&trade; 62S2 evaluation kit (CY8CEVAL-062S2-LAI-4373M2)](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ceval-062s2/)

## Hardware setup
This application uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

The test setup are shown below:

![](images/unified-app-setup.jpg)

**Note:** The PSoC&trade; 6 Bluetooth&reg; LE pioneer kit (CY8CKIT-062-BLE) and the PSoC&trade; 6 Wi-Fi Bluetooth&reg; pioneer kit (CY8CKIT-062-WIFI-BT) ship with KitProg2 installed. ModusToolbox software requires KitProg3. Before using this application, make sure that the board is upgraded to KitProg3. The tool and instructions are available in the Firmware Loader GitHub repository. If you do not upgrade, you will see an error like "unable to find CMSIS-DAP device" or "KitProg firmware is out of date".

## Wi-Fi Cert Test bed setup
Refer to https://github.com/Infineon/mtb-anycloud-wifi-cert-tester#wi-fi-cert-test-bed-setup

## Software setup
- Install a terminal emulator such as Tera Term, PuTTY, or MiniCom to input the command and observe the output.
- Requires the WL tool running on a Linux PC and uses the UART port for communication with the target. The pre-built executables for the WL tool are available in the *wl-tool-bin/* directory.
- The Wi-Fi Cert Test bed uses Python scripts to take control of the terminal.

## Using the application
Create the project and open it using one of the following:
<details><summary><b>In command-line interface (CLI)</b></summary>

ModusToolbox&trade; software provides the Project Creator as both a GUI tool and a command line tool, "project-creator-cli". The CLI tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the "project-creator-cli" tool. On Windows, use the command line "modus-shell" program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools. You can access it by typing `modus-shell` in the search box in the Windows menu. In Linux and macOS, you can use any terminal application.

The following table lists the arguments for this tool:

Argument | Description | Required/optional
---------|-------------|-----------
`--board-id` | Defined in the `<id>` field of the [BSP](https://github.com/Infineon?q=bsp-manifest&type=&language=&sort=) manifest | Required
`--app-id`   | Defined in the `<id>` field of the [CE](https://github.com/Infineon?q=ce-manifest&type=&language=&sort=) manifest | Required
`--target-dir`| Specify the directory in which the application is to be created if you prefer not to use the default current working directory | Optional
`--user-app-name`| Specify the name of the application if you prefer to have a name other than the example's default name | Optional


The following example will clone the "[Wifi Mfg Tester](https://github.com/Infineon/mtb-anycloud-wifi-mfg-tester)" application with the desired name "MyWifiMfgTester" configured for the *CY8CKIT-062-WIFI-BT* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id CY8CKIT-062-WIFI-BT --app-id mtb-anycloud-wifi-mfg-tester --user-app-name MyWifiMfgTester --target-dir "C:/mtb_projects"
   ```

**Note:** The project-creator-cli tool uses the `git clone` and `make getlibs` commands to fetch the repository and import the required libraries. For more details, see the "Project creator tools" section of the [ModusToolbox&trade; software user guide](https://www.cypress.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>

<details><summary><b>In third-party IDEs</b></summary>

Use one of the following options:

- **Use the standalone [Project Creator](http://www.cypress.com/ModusToolboxProjectCreator) tool:**

   1. Launch Project Creator from the Windows Start menu or from *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/project-creator.exe*.

   2. In the initial **Choose Board Support Package** screen, select the BSP and click **Next**.

   3. In the **Select Application** screen, select the appropriate IDE from the **Target IDE** drop-down menu.

   4. Click **Create** and follow the instructions printed in the bottom pane to import or open the exported project in the respective IDE.

- **Use command-line interface (CLI):**

1. Follow the instructions from the **In command-line interface (CLI)** section to create the application, and import the libraries using the `make getlibs` command.

2. Export the application to a supported IDE using the `make <ide>` command.

3. Follow the instructions displayed in the terminal to create or import the application as an IDE project.

For a list of supported IDEs and more details, see the "Exporting to IDEs" section of the [ModusToolbox&trade; software user guide](https://www.cypress.com/ModusToolboxUserGuide) (locally available at *{ModusToolbox&trade; install directory}/docs_{version}/mtb_user_guide.pdf*).

</details>


## Operation
### wifi-bt-tester
Refer to https://github.com/Infineon/mtb-wifi-bluetooth-tester#operation

### wifi-mfg-tester
1. Go to the WL tool directory (Using the Linux version of wl tool is recommended):
   ```
   cd wl-tool-bin/Linux
   ```

2. Reset the board by pressing the SW1 button.

3. Run the command on Linux host for the WLAN chip on the target board:

   ```
   wl --serial <port> ver`
   ```

   For example:

   ```
   sudo ./wl --serial /dev/ttyACM0 ver
   ```

4. Observe the output of the command.

   The list of WL commands which can be retrieved by typing `--help`. Partial output of the command and display is as follows:
   ```
   # sudo ./wl --serial /dev/ttyACM0 --help

   Usage: wl [-a|i <adapter>] [-h] [-d|u|x] <command> [arguments]

		-h        this message and command descriptions
		-h [cmd]  command description for cmd
		-a, -i    adapter name or number
		-d        output format signed integer
		-u        output format unsigned integer
		-x        output format hexadecimal

		ver     get version information

		cmds    generate a short list of available commands

		ioctl_echo
			check ioctl functionality

		up      reinitialize and mark adapter up (operational)

		down    reset and mark adapter down (disabled)

		out     mark adapter down but do not reset hardware(disabled)
				On dual-band cards, cards must be band-locked before use.

		clk     set board clock state. return error for set_clk attempt if the driver is not down
				0: clock off
				1: clock on

		restart Restart driver.  Driver must already be down.

		reboot  Reboot platform

   ```
### wifi-cert-tester
Refer to https://github.com/Infineon/mtb-anycloud-wifi-cert-tester#operation

### wifi-wlan-lowpower
1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector.

2. Enable LPA_ENABLE marco in Makefile.

3. Connect your PC to the Wi-Fi AP.

4. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 115200 baud.

5. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE for ModusToolbox&trade; software</b></summary>

      1. Select the application project in the Project Explorer.

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**.
   </details>

   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target. The default toolchain is specified in the application's Makefile but you can override this value manually:
      ```
      make program TOOLCHAIN=<toolchain>
      ```

      Example:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

6. After programming, the application starts automatically and suspends the network stack.

   **Figure 1. Power on and suspend the network stack**

   ![](images/figure1.png)

7. Connect DUT to the same Wi-Fi AP that already connected to your PC in Step 3. The DUT will resume network stack, connected to the AP and then suspends the network stack again.

   **Figure 2. Connected to AP and suspend the network stack**

   ![](images/figure2.png)

8. Open PC command prompt and ping the IP address displayed on the serial terminal:

    ```
    ping <IP address>
    ```

   The network stack resumes. If WHD_PRINT_DISABLE marco is enabled in Makefile, the device displays the deep sleep and Wi-Fi SDIO bus statistics on the terminal.

   **Note:** The host MCU will wake up when any network activity is detected and not necessarily due to the ping from the PC. The reasons for network activity could be due to the broadcast or multicast packets issued by the AP. Further power saving can be done by employing offload features like packet filtering, which will increase the time the host MCU will be in deep sleep. See [AN227910 - Low-power system design with AIROC&trade; CYW43012 Wi-Fi & Bluetooth&reg; combo chip and PSoC&trade; 6 MCU](https://www.infineon.com/dgdl/Infineon-AN227910_Low-power_system_design_with_AIROC_CYW43012_Wi-Fi_%26_Bluetooth_combo_chip_and_PSoC_6_MCU-ApplicationNotes-v03_00-EN.pdf?fileId=8ac78c8c7cdc391c017d0d39b66166f3) for more details.

    **Figure 3. Resuming the network stack**

    ![](images/figure3.png)

   See [Measuring the current consumption](https://github.com/Infineon/mtb-example-wifi-wlan-lowpower#measuring-the-current-consumption) for instructions on how to measure the current consumed by the PSoC&trade; 6 MCU and the Wi-Fi device.

9. Refer to https://github.com/Infineon/mtb-example-wifi-wlan-lowpower for more detailed information

------

All other trademarks or registered trademarks referenced herein are the property of their respective owners.

------

![Banner](images/Banner.png)

-------------------------------------------------------------------------------

© Cypress Semiconductor Corporation, 2020-2021. This document is the property of Cypress Semiconductor Corporation, an Infineon Technologies company, and its affiliates ("Cypress").  This document, including any software or firmware included or referenced in this document ("Software"), is owned by Cypress under the intellectual property laws and treaties of the United States and other countries worldwide.  Cypress reserves all rights under such laws and treaties and does not, except as specifically stated in this paragraph, grant any license under its patents, copyrights, trademarks, or other intellectual property rights.  If the Software is not accompanied by a license agreement and you do not otherwise have a written agreement with Cypress governing the use of the Software, then Cypress hereby grants you a personal, non-exclusive, nontransferable license (without the right to sublicense) (1) under its copyright rights in the Software (a) for Software provided in source code form, to modify and reproduce the Software solely for use with Cypress hardware products, only internally within your organization, and (b) to distribute the Software in binary code form externally to end users (either directly or indirectly through resellers and distributors), solely for use on Cypress hardware product units, and (2) under those claims of Cypress’s patents that are infringed by the Software (as provided by Cypress, unmodified) to make, use, distribute, and import the Software solely for use with Cypress hardware products.  Any other use, reproduction, modification, translation, or compilation of the Software is prohibited.
<br>
TO THE EXTENT PERMITTED BY APPLICABLE LAW, CYPRESS MAKES NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, WITH REGARD TO THIS DOCUMENT OR ANY SOFTWARE OR ACCOMPANYING HARDWARE, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.  No computing device can be absolutely secure.  Therefore, despite security measures implemented in Cypress hardware or software products, Cypress shall have no liability arising out of any security breach, such as unauthorized access to or use of a Cypress product.  CYPRESS DOES NOT REPRESENT, WARRANT, OR GUARANTEE THAT CYPRESS PRODUCTS, OR SYSTEMS CREATED USING CYPRESS PRODUCTS, WILL BE FREE FROM CORRUPTION, ATTACK, VIRUSES, INTERFERENCE, HACKING, DATA LOSS OR THEFT, OR OTHER SECURITY INTRUSION (collectively, "Security Breach").  Cypress disclaims any liability relating to any Security Breach, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any Security Breach.  In addition, the products described in these materials may contain design defects or errors known as errata which may cause the product to deviate from published specifications.  To the extent permitted by applicable law, Cypress reserves the right to make changes to this document without further notice. Cypress does not assume any liability arising out of the application or use of any product or circuit described in this document.  Any information provided in this document, including any sample design information or programming code, is provided only for reference purposes.  It is the responsibility of the user of this document to properly design, program, and test the functionality and safety of any application made of this information and any resulting product.  "High-Risk Device" means any device or system whose failure could cause personal injury, death, or property damage.  Examples of High-Risk Devices are weapons, nuclear installations, surgical implants, and other medical devices.  "Critical Component" means any component of a High-Risk Device whose failure to perform can be reasonably expected to cause, directly or indirectly, the failure of the High-Risk Device, or to affect its safety or effectiveness.  Cypress is not liable, in whole or in part, and you shall and hereby do release Cypress from any claim, damage, or other liability arising from any use of a Cypress product as a Critical Component in a High-Risk Device.  You shall indemnify and hold Cypress, including its affiliates, and its directors, officers, employees, agents, distributors, and assigns harmless from and against all claims, costs, damages, and expenses, arising out of any claim, including claims for product liability, personal injury or death, or property damage arising from any use of a Cypress product as a Critical Component in a High-Risk Device.  Cypress products are not intended or authorized for use as a Critical Component in any High-Risk Device except to the limited extent that (i) Cypress’s published data sheet for the product explicitly states Cypress has qualified the product for use in a specific High-Risk Device, or (ii) Cypress has given you advance written authorization to use the product as a Critical Component in the specific High-Risk Device and you have signed a separate indemnification agreement.
<br>
Cypress, the Cypress logo, and combinations thereof, WICED, ModusToolBox, PSoC, CapSense, EZ-USB, F-RAM, and Traveo are trademarks or registered trademarks of Cypress or a subsidiary of Cypress in the United States or in other countries.  For a more complete list of Cypress trademarks, visit cypress.com.  Other names and brands may be claimed as property of their respective owners.
