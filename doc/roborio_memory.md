# Reducing Memory Usage on the RoboRio v1

The RoboRio v1 only has 256MiB of RAM. The JVM used to run the robot code tends to consume most if not all of the available memory which can lead to out of memory issues. There are a couple of services that we can disable to make more memory available and reduce the occurrence of out of memory errors.

**NOTE:** The following configuration changes are not officially supported and may lead to other issues.

## Disable the LabView Runtime

The image provided by NI supports using C++, Java and LabView for robot code development. C++ and Java use runtimes that are active when the robot code is running, but the LabView runtime runs all of the time. Since we do not use LabView, this consumes memory unnecessarily. We can disable the LabView runtime by replacing the `/usr/local/natinst/labview/lvrt` binary with the following [bash script][LVRTSCRIPT]:

```bash
#!/bin/sh
exec 2>&1 >/dev/null
while true; do
  APP_PATH=`nirtcfg --file /etc/natinst/share/lvrt.conf --get section=LVRT,token=RTTarget.ApplicationPath`
  if [ "$APP_PATH" = /home/lvuser/natinst/bin/TBLStartupApp.rtexe ]
  then
    APP_BOOT=`nirtcfg --file /etc/natinst/share/lvrt.conf --get section=LVRT,token=RTTarget.LaunchAppAtBoot,value=true | tr "[:upper:]" "[:lower:]"`
    APP_DISABLED=`nirtcfg --get section=SYSTEMSETTINGS,token=NoApp.enabled,value=false | tr "[:upper:]" "[:lower:]"`
    if [ "$APP_BOOT" = true ] && [ "$APP_DISABLED" = false ]
    then
      /usr/local/frc/bin/frcRunRobot.sh
    fi
  else
    exec -a lvrt ./ni-lvrt
  fi
  sleep 1
done
```

To replace the script, [SSH to the RoboRio] and take the following steps:

1. Rename `lvrt` to `ni-lvrt`:

    ```sh
    mv /usr/local/natinst/labview/lvrt /usr/local/natinst/labview/ni-lvrt
    ```

2. Create the `lvrt` script file using `vi`:

    ```sh
    vi /usr/local/natinst/labview/lvrt
    ```

3. Copy the script above and paste it into the `vi` editor. (HINT: Press `a` to enter append mode and then right-click the PuTTY window.)
4. Save the script. (HINT: Press ESC to exit append mode, then type `:w!` and ENTER to save the file and `:q!` and ENTER to quit `vi`.)
5. Make the script executable:

    ```sh
    chmod +x /usr/local/natinst/labview/lvrt
    ```

6. Reboot the RoboRio:

    ```sh
    reboot
    ```

## Disable NI Web Server

The NI image provides a web interface to inspect RoboRio configuration.

To disable the NI Web Server, [SSH to the RoboRio] and take the following steps:

1. Delete the file `/etc/rc5.d/S25systemWebServer`.

    ```sh
    rm /etc/rc5.d/S25systemWebServer
    ```

2. Reboot the RoboRio:

    ```sh
    reboot
    ```

[LVRTSCRIPT]: https://gist.github.com/PeterJohnson/2702b331ee3c236188ec76ecf499c333
[SSH to the RoboRio]: https://docs.wpilib.org/en/stable/docs/software/roborio-info/roborio-ssh.html#ssh
