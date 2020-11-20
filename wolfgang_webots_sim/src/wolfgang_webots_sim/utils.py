import os
import time


def fix_webots_folder(sim_proc_pid):
    # Fix for webots folder name on some systems
    time.sleep(1)  # Wait for webots
    for folder in os.listdir('/tmp'):
        if folder.startswith(f'webots-{sim_proc_pid}-'):
            try:
                os.remove(F'/tmp/webots-{sim_proc_pid}')
            except FileNotFoundError:
                pass
            os.symlink(F'/tmp/{folder}', F'/tmp/webots-{sim_proc_pid}')
