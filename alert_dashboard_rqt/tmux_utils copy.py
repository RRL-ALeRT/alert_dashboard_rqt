import subprocess
import paramiko


class TmuxSSHWrapper:
    def __init__(self, username, ip, session_name):
        self.username = username
        self.ip = ip
        self.session_name = session_name

        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())

    def __del__(self):
        self.ssh.close()

    def _execute_ssh_command(self, command):
        try:
            self.ssh.connect(self.ip, username=self.username, timeout=0.5)
            stdin, stdout, stderr = self.ssh.exec_command(command)
            return stdout.read().decode().strip()
        except Exception as e:
            print(f"Error executing command: {e}")

    def new_session(self):
        command = f"tmux new-session -s {self.session_name} -d"
        self._execute_ssh_command(command)

    def new_window(self, window_name):
        if window_name in self.get_active_windows():
            return
        command = f"tmux new-window -t {self.session_name}: -n {window_name}"
        self._execute_ssh_command(command)

    def temporary_window(self, window_name, command, delay=1):
        self.new_window(window_name)
        command += f" ; tmux kill-window -t {self.session_name}:{window_name}"
        self.window_command(window_name, command, delay)

    def kill_window(self, window_name):
        command = f"tmux kill-window -t {self.session_name}:{window_name}"
        self._execute_ssh_command(command)

    def window_command(self, window_name, command, delay=0):
        command = f"sleep {delay} && tmux send-keys -t {self.session_name}:{window_name} '{command}' Enter"
        self._execute_ssh_command(command)

    def window_cancel(self, window_name):
        command = f"tmux send-keys -t {self.session_name}:{window_name} C-c"
        self._execute_ssh_command(command)

    def window_select(self, window_name):
        command = f"tmux select-window -t {self.session_name}:{window_name}"
        self._execute_ssh_command(command)

    def get_active_windows(self):
        command = f"tmux list-windows -t {self.session_name}"
        output = self._execute_ssh_command(command)
        if output == None:
            return []
        return [
            line.split(" ")[1].split("*")[0].split("#")[0].split("-")[0]
            for line in output.split("\n")
            if ":" in line
        ]


class TmuxLocalWrapper:
    def __init__(self, session_name):
        self.session_name = session_name

    def new_session(self):
        ssh_command = f"tmux new-session -s {self.session_name} -d"
        subprocess.Popen(ssh_command, shell=True)

    def new_window(self, window_name):
        if window_name in self.get_active_windows():
            return
        ssh_command = f"tmux new-window -t {self.session_name}: -n {window_name}"
        subprocess.Popen(ssh_command, shell=True)

    def temporary_window(self, window_name, command, delay=1):
        self.new_window(window_name)
        command += f" ; tmux kill-window -t {self.session_name}:{window_name}"
        self.window_command(window_name, command, delay)

    def kill_window(self, window_name):
        ssh_command = f"tmux kill-window -t {self.session_name}:{window_name}"
        subprocess.Popen(ssh_command, shell=True)

    def window_command(self, window_name, command, delay=0):
        ssh_command = f"sleep {delay} && tmux send-keys -t {self.session_name}:{window_name} '{command}' \C-m"
        subprocess.Popen(ssh_command, shell=True)

    def window_cancel(self, window_name):
        ssh_command = f"tmux send-keys -t {self.session_name}:{window_name} \C-c"
        subprocess.Popen(ssh_command, shell=True)

    def window_select(self, window_name):
        ssh_command = f"tmux select-window -t {self.session_name}:{window_name}"
        subprocess.Popen(ssh_command, shell=True)

    def get_active_windows(self):
        ssh_command = f"tmux list-windows -t {self.session_name}"

        result = subprocess.Popen(
            [ssh_command],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            shell=True,
        )

        output = result.stdout.read().decode()
        output = output.split("\n")

        return [
            line.split(" ")[1].split("*")[0].split("#")[0].split("-")[0]
            for line in output.split("\n")
            if ":" in line
        ]
