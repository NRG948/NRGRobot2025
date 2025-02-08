# NRGRobot2025

The robot code of FRC Team 948 - Newport Robotics Group for FIRST Robotics Competition 2025 REEFSCAPE presented by Haas game.

## Setup

1. Install Git from <https://git-scm.org>.

2. Follow the instructions at [WPILib Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) to install the FRC development environment.

3. Clone the repository.

    * Windows

        ```powershell
        git clone https://github.com/NRG948/NRGRobot2025.git
        ```

    * Linux/MacOS

        * Create an SSH key if you do not already have one.

            ```sh
            ssh-keygen -t rsa
            ```

            Press ENTER to accept the default location.

            Enter a passphrase and press ENTER, or simply press ENTER to use no passphrase.

        * Clone the repository.

            ```sh
            git clone git@github.com:NRG948/NRGRobot2025.git
            ```

4. [Create a personal access token](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/creating-a-personal-access-token) with only  `packages:read` scope.

5. Add the following entries to `$HOME/.gradle/gradle.properties` replacing `USERNAME` with your GitHub user name and `TOKEN` with the token value itself.

    ```properties
    gpr.user=USERNAME
    gpr.key=TOKEN
    ```

## Build

To build the robot code, use the "WPILib: Build Robot Code" command from the ![WPILib](doc/image/wpilib-24.png) menu in Visual Studio Code, or run the following command from a command-line:

* Windows

    ```powershell
    ./gradlew build -Dorg.gradle.java.home="C:\Users\Public\wpilib\2025\jdk" 
    ```

* Linux/MacOS

    ```sh
    ./gradlew build -Dorg.gradle.java.home="~/wpilib/2025/jdk" 
    ```

> **NOTE:** When using external library SNAPSHOTs, you may need to explicitly refresh the Gradle build cache to pickup changes. In that case, include the `--refresh-dependencies` option in in your build command.

## Deploy

To deploy the robot code, use the "WPILib: Deploy Robot Code" command from the ![WPILib](doc/image/wpilib-24.png) menu in Visual Studio Code, or run the following command from a command-line:

* Windows

    ```powershell
    PS> ./gradlew deploy -Dorg.gradle.java.home="C:\Users\Public\wpilib\2025\jdk" 
    ```

* Linux/MacOS

    ```sh
    ./gradlew deploy -Dorg.gradle.java.home="~/wpilib/2025/jdk" 
    ```
