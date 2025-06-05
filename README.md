#     Project_SoSe2021_TeamSecurity

This repository is one of the embedded security showcases carried out in a team as a part of my Master's course at Hochschule Darmstadt during Sommer-Semester 2021.
The usecase aims at demonstrating secured communication between an Embedded Client and a Server via Bluetooth Low Energy (BLE). AES-128 is used for encryption and decrption

The repository is based on Infineon's Code examples for ModusToolbox™ software

BLE_Server is based on [AnyCloud BLE CTS Server](https://github.com/Infineon/mtb-example-anycloud-ble-cts-server/tree/release-v1.0.0)

BLE_Cleint is based on [AnyCloud BLE CTS Client](https://github.com/Infineon/mtb-example-anycloud-ble-cts-client/tree/release-v1.0.0)

## Hardware
Two boards [CY8CKIT-064S0S2-4343W](https://www.infineon.com/cms/en/product/evaluation-boards/cy8ckit-064s0s2-4343w/?utm_source=cypress&utm_medium=referral&utm_campaign=202110_globe_en_all_integration-dev_kit) are used for the demonstration - one as a client and the other as server

## IDE
[ModusToolbox™ software](https://www.cypress.com/products/modustoolbox-software-environment) v2.3 + v2.3.1 (patch)