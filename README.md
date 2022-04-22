# FTC Fission 310 2021-2022 Freight Frenzy
Managed by Lucas Lee and Paul Serbanescu

## Installation
- Download Java 8 by following these [instructions](https://dojo.stuycs.org/resources/software_installation_and_tips/installation_instructions/programming_languages/installing_jdk.html)
- Download [Android Studio](https://developer.android.com/studio)
- Follow the [road-runner template installation](https://github.com/Fission310/road-runner-template#installation)
- In Android Studio:
  - `File > Open... > road-runner-template`
  - Perform a test build by going to `Build > Make Project`, pressing `⌘F9`, or finding the green hammer in the toolbar

## Deploying code to the robot
### Using a REV Control Hub
- Make sure the control hub is powered
- Follow REV setup instructions starting [here](https://docs.revrobotics.com/rev-control-system/control-hub-gs)
- Connect to the control hub's wifi
- Navigate to http://192.168.43.1:8080 in your web browser to view the control hub's status
- Inside Android Studio, open the terminal by clicking on it in the bottom toolbar or pressing `⌥F12`
  - Type `adb connect 192.168.43.1:5555`
  - You should now be connected to the control hub and able to wirelessly push code
    <img width="395" alt="image" src="https://user-images.githubusercontent.com/61565464/164758656-5a7678cf-33ec-4916-a37f-d24263afad52.png">
  - Press the green play button or `^R`
  #### Common Issues:
  - If your computer loses wifi connection from the control hub and regains it at a later point `adb connect` may not work to reconnect to the control hub.
  - In that case, type `adb kill-server` to reset the adb server (this may take a couple minutes). Afterwards, `adb connect` again

## FissionLib
This project uses [FissionLib](https://github.com/Fission310/FissionLib), our FTC library.
