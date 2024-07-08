import warnings
from cryptography.utils import CryptographyDeprecationWarning
with warnings.catch_warnings():
    warnings.filterwarnings('ignore', category=CryptographyDeprecationWarning)
    import paramiko
    # https://github.com/paramiko/paramiko/issues/2038#issuecomment-1113368347

import subprocess
import time

class TmuxWrapperBase:
    def __init__(self, session_name):
        self.session_name = session_name
        self.ssh = None

    def _execute_command(self, command):
        if self.ssh:  # If ssh client is set, execute remotely
            try:
                _, stdout, stderr = self.ssh.exec_command(command, get_pty=True)
                return stdout.read().decode().strip()
            except paramiko.SSHException as e:
                print(f"Error executing command: {e}")
        else:  # Execute locally
            try:
                result = subprocess.run(command, shell=True, capture_output=True, text=True)
                return result.stdout.strip()
            except subprocess.CalledProcessError as e:
                print(f"Error executing command: {e}")
        return None

    def new_session(self):
        command = f"tmux new-session -s {self.session_name} -d"
        self._execute_command(command)

    def new_window(self, window_name):
        if window_name in self.get_active_windows():
            return
        command = f"tmux new-window -t {self.session_name}: -n {window_name}"
        self._execute_command(command)

    def temporary_window(self, window_name, command, delay=1):
        self.new_window(window_name)
        command += f" ; tmux kill-window -t {self.session_name}:{window_name}"
        self.window_command(window_name, command, delay)

    def kill_window(self, window_name):
        command = f"tmux kill-window -t {self.session_name}:{window_name}"
        self._execute_command(command)

    def window_command(self, window_name, command, delay=0):
        command = f"sleep {delay} && tmux send-keys -t {self.session_name}:{window_name} '{command}' Enter"
        self._execute_command(command)

    def window_cancel(self, window_name):
        command = f"tmux send-keys -t {self.session_name}:{window_name} C-c"
        self._execute_command(command)

    def window_select(self, window_name):
        command = f"tmux select-window -t {self.session_name}:{window_name}"
        self._execute_command(command)

    def get_active_windows(self):
        command = f"tmux list-windows -t {self.session_name}"
        output = self._execute_command(command)
        return [] if output is None else [
            line.split(" ")[1].split("*")[0].split("#")[0].split("-")[0]
            for line in output.split("\n")
            if ":" in line
        ]


class TmuxSSHWrapper(TmuxWrapperBase):
    def __init__(self, username, ip, session_name):
        super().__init__(session_name)
        self.username = username
        self.ip = ip
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        while True:
            try:
                self.ssh.connect(self.ip, username=self.username, timeout=0.5)
                break
            except TimeoutError as e:
                print(f"Error connecting SSH {e}")
            except paramiko.ssh_exception.SSHException as e:
                print(f"Error connecting SSH {e}")
                time.sleep(1)

    def __del__(self):
        if self.ssh:
            self.ssh.close()


class TmuxLocalWrapper(TmuxWrapperBase):
    def __init__(self, session_name):
        super().__init__(session_name)


if __name__ == "__main__":
    # Example Usage
    ssh_wrapper = TmuxSSHWrapper(username="your_username", ip="your_ip", session_name="session_name")
    ssh_wrapper.new_session()
    ssh_wrapper.new_window("window1")
    ssh_wrapper.temporary_window("window2", "ls", delay=2)
    active_windows = ssh_wrapper.get_active_windows()
    print("Active Windows:", active_windows)
    ssh_wrapper.__del__()

    local_wrapper = TmuxLocalWrapper(session_name="local_session")
    local_wrapper.new_session()
    local_wrapper.new_window("local_window1")
    local_wrapper.window_command("local_window1", "echo 'Hello World'")
    local_wrapper.kill_window("local_window1")
    active_windows_local = local_wrapper.get_active_windows()
    print("Active Local Windows:", active_windows_local)
