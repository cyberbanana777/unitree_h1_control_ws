import subprocess
import os
from ament_index_python.packages import get_package_share_directory

def main():
    package_share_dir = get_package_share_directory('hands_init')

    executable_path = os.path.join(package_share_dir, 'inspire_hand')
    cmd = ['sudo', '-S', executable_path, '-s', '/dev/ttyUSB0']

    sudo_password = "Unitree0408\n"  # пароль с переводом строки

    try:
        with subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                              text=True, preexec_fn=os.setsid) as proc:
            # Передаём пароль в stdin
            proc.stdin.write(sudo_password)
            proc.stdin.flush()

            for line in proc.stdout:
                print(line, end='')

            proc.wait()
            print(f"Процесс завершился с кодом {proc.returncode}")

    except KeyboardInterrupt:
        print("\nПолучен Ctrl+C! Завершаем процесс...")

        try:
            kill_cmd = ['sudo', '-S', 'kill', '-TERM', str(proc.pid)]
            kill_proc = subprocess.Popen(kill_cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        text=True)
            out, _ = kill_proc.communicate(sudo_password)
            if kill_proc.returncode == 0:
                print("Процесс успешно завершён через sudo kill.")
            else:
                print(f"Ошибка при завершении процесса, код: {kill_proc.returncode}")
                print(out)
        except Exception as e:
            print(f"Ошибка при попытке завершить процесс: {e}")


if __name__ == '__main__':
    main()