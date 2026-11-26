# Copy a motion YAML file with SCP

The student PC uses `send_social_motion.py` to copy a motion YAML file to the teacher PC over SSH. The classroom configuration uses the account `biorob` at `192.168.1.5` on port 22.

## Set up SSH on the teacher PC

Run these commands on the Ubuntu host (PC6), outside Docker.

```bash
sudo apt update
sudo apt install openssh-server
sudo systemctl enable --now ssh
```

From the teacher host, verify that the destination directory exists and that `biorob` can write to it.

```bash
motion_dir=/home/biorob/Desktop/UR5e_social_robotics/install/social_robot_behaviors/share/social_robot_behaviors/motions
ls -ld "$motion_dir"
sudo -u biorob test -w "$motion_dir" && echo "Destination is writable"
```

The destination is under the workspace's `install/` directory. If it is missing, build and install `social_robot_behaviors` in the teacher workspace first. If the directory exists but `biorob` cannot write to it, set its owner on the teacher host.

```bash
sudo chown biorob:biorob "$motion_dir"
```

## Copy a file from the student PC

The student PC needs Python 3 and the OpenSSH client, which provides both `ssh` and `scp`. If either SSH command is missing on Ubuntu, install the client with `sudo apt install openssh-client`.

1. Test the connection with `ssh biorob@192.168.1.5`. If prompted for a password, use the password for the `biorob` account on the teacher PC. Run `exit` after logging in to return to the student terminal.
2. Save the YAML file in `/home/student/Desktop/UR5e_social_robotics/src/social_robot_behaviors/motions/`.
3. Run the script on the student PC.

   ```bash
   cd /home/student/Desktop/UR5e_social_robotics
   python3 Documentation/Files/Send_motion/send_social_motion.py
   ```

4. At the `Nom del fitxer YAML:` prompt, enter the filename including its extension, for example `wave.yaml`. The script prints `Fitxer copiat correctament.` when the copy succeeds.

## Troubleshooting

- **APT reports conflicting `Signed-By` values for ROS 2.** On PC6, this happened because both `/etc/apt/sources.list.d/ros2.list` and `/etc/apt/sources.list.d/ros2.sources` defined the same ROS 2 repository. If both are present and point to that repository, keep the `ros2.sources` entry managed by `ros2-apt-source` and move the duplicate aside. Then retry the OpenSSH installation.

  ```bash
  sudo mv /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.list.disabled
  sudo apt update
  ```

- **`Connection refused`.** On the teacher host, check the SSH service with `systemctl status ssh` and confirm the teacher IP with `ip -br addr`. If the classroom IP has changed, update `PROFESSOR_IP` in the student script.
- **`Permission denied, please try again` or `Permission denied (publickey,password)`.** SSH rejected the login. Test with `ssh biorob@192.168.1.5` and check the teacher account credentials.
- **`scp: dest open "...": Permission denied`.** SSH login succeeded, but SCP could not write the file. Check the destination directory as shown above. If a file with the same name already exists, also check that `biorob` can write to that file on the teacher host.
