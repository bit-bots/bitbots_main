#!/usr/bin/env python3

import argparse
import time
from pathlib import Path
import os
import sys
import subprocess
import yaml
import re

cwd = str(Path(os.path.abspath(__file__)).parents[1])

workspaces = {
    'wolfgang': '/home/bitbots/wolfgang_ws',
    'davros': '/home/bitbots/davros_ws'
}


def print_err(msg):
    print('\033[91m\033[1m#################   ' + msg + '   #################\033[0m')


def print_warn(msg):
    print('\033[93m\033[1m#################   ' + msg + '   #################\033[0m')


def print_success(msg):
    print('\033[92m\033[1m' + msg + '\033[0m')


def print_info(msg):
    print('\033[1m' + msg + '\033[0m')


def get_includes_from_file(file, package=None):
    includes = list()
    with open(file) as f:
        data = yaml.safe_load(f)
        for entry in data['exclude']:
            # --include is right here. No worries.
            includes.append('--include=- {}'.format(entry))
        for entry in data['include']:
            if type(entry) == dict:
                for folder, subfolders in entry.items():
                    if package is None:
                        includes.append('--include=+ {}'.format(folder))
                        for subfolder in subfolders:
                            includes.append('--include=+ {}/{}'.format(folder, subfolder))
                            includes.append('--include=+ {}/{}/**'.format(folder, subfolder))
                    elif package in subfolders:
                        includes.append('--include=+ {}'.format(folder))
                        includes.append('--include=+ {}/{}'.format(folder, package))
                        includes.append('--include=+ {}/{}/**'.format(folder, package))
            elif type(entry) == str:
                if package is None or entry == package:
                    includes.append('--include=+ {}'.format(entry))
                    includes.append('--include=+ {}/**'.format(entry))
    includes.append('--include=- *')
    return includes


def add_game_controller_config(bot_id, workspace, host):
    team_id = 18
    config_path = workspace + '/src/humanoid_league_misc/humanoid_league_game_controller/config/game_controller.yaml'
    r = subprocess.run([
        'ssh',
        'bitbots@{}'.format(host[0]),
        'echo -e "team_id: {}\nbot_id: {}" > {}'.format(team_id, bot_id, config_path)
    ])
    if r.returncode != 0:
        print_err('Adding a game controller config failed!')
        sys.exit(r.returncode)
    print_info('A game controller config with team_id {} and bot_id {} was added.'.format(team_id, bot_id))


def clean_src_dir(host, workspace):
    print_info('Cleaning source directory on {}...'.format(host[1]))
    call = [
        'ssh',
        'bitbots@{}'.format(host[0]),
        'rm -rf {}/src'.format(workspace)
    ]
    clean_result = subprocess.run(call)
    if clean_result.returncode != 0:
        print_err('Cleaning the source directory failed!')
        sys.exit(clean_result.returncode)


def synchronize(sync_includes_file, host, workspace, package=None):
    call = [
        'rsync',
        '-ca',
        '' if args.quiet else '-v',
        '--delete']
    call.extend(get_includes_from_file(os.path.join(cwd, sync_includes_file), package))
    call.extend([
        cwd + '/',
        'bitbots@{}:{}/src/'.format(host[0], workspace)])
    sync_result = subprocess.run(call)
    if sync_result.returncode != 0:
        print_err('Synchronizing the workspace failed!')
        sys.exit(sync_result.returncode)

    if host[1].startswith('jetson'):
        call = ['ssh',
                'bitbots@{}'.format(host[0]),
                'sed -i "/camera_name/s/ROBOT/{}/" ~/wolfgang_ws/src/wolves_image_provider/config/camera_settings.yaml'.format(robot_name_name)]
        camera_result = subprocess.run(call)
        if camera_result.returncode != 0:
            print_err('Configuring the camera calibration failed!')
            sys.exit(camera_result.returncode)


# noinspection PyUnboundLocalVariable
def configure(args):
    start_motion = args.yes_to_all or input('Start motion on boot? (Y/n) ').lower() != 'n'
    start_behaviour = args.yes_to_all or input('Start behaviour on boot? (Y/n) ').lower() != 'n'
    start_vision = args.yes_to_all or input('Start vision on boot? (Y/n) ').lower() != 'n'
    start_roscore = args.yes_to_all or input('Start roscore on boot? (Y/n) ').lower() != 'n'

    for host in hosts:
        if host[1].startswith('odroid') or host[1].startswith('nuc'):
            add_game_controller_config(host[1][-1:], workspace, host)

        data = {
            'roscore': '',
            'motion': '',
            'behavior': '',
            'vision': ''
        }
        
        if host[0].startswith('nuc'):
            data['motion'] = 'sudo /bin/systemctl start bitbots_motion.service; sudo /bin/systemctl enable bitbots_motion.service; ' \
                if start_motion else \
                'sudo /bin/systemctl disable bitbots_motion.service; '

            data['behavior'] = 'sudo /bin/systemctl start bitbots_highlevel.service; sudo /bin/systemctl enable bitbots_highlevel.service; ' \
                if start_behaviour else \
                'sudo /bin/systemctl disable bitbots_highlevel.service; '

            data['roscore'] = 'sudo /bin/systemctl start roscore.service; sudo /bin/systemctl enable roscore.service; ' \
                if start_roscore else \
                'sudo /bin/systemctl disable roscore.service; '

        elif host[0].startswith('jetson'):
            data['vision'] = 'sudo /bin/systemctl start bitbots_vision.service; sudo /bin/systemctl start bitbots_vision.service; ' \
                if start_vision else \
                'sudo /bin/systemctl disable bitbots_vision.service; '

        print_info('Configuring boot for {}...'.format(host[1]))
        r = subprocess.run([
            'ssh',
            'bitbots@{}'.format(host[0]),
            'bash -c \'{roscore}{motion}{behavior}{vision}\''.format(**data)
        ])
        if r.returncode != 0:
            print_err('Configuring {} failed!'.format(host[1]))
            exit(r.returncode)

        print_info('Calling game_settings on host {}'.format(host[1]))
        with subprocess.Popen([
            'ssh', 'bitbots@{}'.format(host[0]),
            'bash -c \'source wolfgang_ws/devel/setup.bash && rosrun bitbots_bringup game_settings.py --no-teamplayer\''
        ], stdout=subprocess.PIPE, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL,
        universal_newlines=True) as proc:
            akk = ''
            skipInput = False
            while proc.returncode is None:
                out = proc.stdout.read(1)
                print(out, end='', flush=True)

                if out == '\n':
                    akk = ''
                    skipInput = False
                else:
                    akk += out

                if not skipInput and re.search(r'Value.*:', akk) is not None:
                    proc.stdin.write(input() + '\n')
                    proc.stdin.flush()
                    skipInput = True
                    time.sleep(1)
                    proc.poll()

    print_success('Successfully configured all robots')


def parse_arguments():
    parser = argparse.ArgumentParser(description='Copy the workspace to the robot and compile it.')
    parser.add_argument('hostname', metavar='hostname', type=str, help='The hostname of the robot (or its name)')
    robot = parser.add_mutually_exclusive_group()
    robot.add_argument('-w', '--wolfgang', action='store_const', help='Compile for wolfgang', const='wolfgang',
                       dest='robot')
    robot.add_argument('-d', '--davros', action='store_const', help='Compile for davros', const='davros', dest='robot')
    mode = parser.add_mutually_exclusive_group(required=False)
    mode.add_argument('-s', '--sync-only', action='store_true', help='Sync files only, don\'t build')
    mode.add_argument('-c', '--compile-only', action='store_true', help='Build only, don\'t copy any files')
    mode.add_argument('-k', '--configure', action='store_true', help='Configure only, don\'t deploy anything new')
    parser.add_argument('-p', '--package', help='Sync/Compile only the given ROS package')
    parser.add_argument('-y', '--yes-to-all', action='store_true', help='Answer yes to all questions')
    parser.add_argument('-j', '--jobs', metavar='N', default=6, type=int, help='Compile using N jobs (default 6)')
    parser.add_argument('--clean-build', action='store_true', help='Clean workspace before building')
    parser.add_argument('--clean-src', action='store_true', help='Clean source directory before building')
    parser.add_argument('--clean-all', action='store_true', help='Clean workspace and source directory before building')
    parser.add_argument('-q', '--quiet', action='store_true', help='Less output')
    parser.set_defaults(robot='wolfgang')
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_arguments()
    hostname = args.hostname

    # Convert names to numbers
    names = {'amy': 1,
             'rory': 2,
             'davros': 5}
    if hostname in names.keys():
        number = names[hostname]
        if args.robot == 'davros':
            hosts = ['odroid{}'.format(number)]
        else:
            hosts = ['{}{}'.format(host, number) for host in ['nuc', 'jetson', 'odroid']]
        robot_name_name = hostname
    else:
        hosts = [hostname]
        robot_name_name = 'ROBOT'
        for name, number in names.items():
            if number == int(hostname[-1]):
                robot_name_name = name
                break

    hosts_tmp = hosts
    # New host array consisting of reachable address (hostname or ip) and hostname
    hosts = []
    for h in hosts_tmp:
        host_result = subprocess.run(['ssh', 'bitbots@{}'.format(h), 'echo $HOST'], stdout=subprocess.PIPE)
        if host_result.returncode != 0:
            print_err('Unable to connect to {}!'.format(h))
        else:
            hosts.append((h, host_result.stdout.decode().strip()))
    if len(hosts) == 0:
        print_err('No hosts available!')
        exit(1)
    if robot_name_name == 'ROBOT':
        # The robot name could not be determined yet
        print(hosts)
        for name, number in names.items():
            if number == int(hosts[0][-1][-1]):
                robot_name_name = name
                break

    if args.robot == 'wolfgang':
        workspace = workspaces['wolfgang']
    elif args.robot == 'davros':
        workspace = workspaces['davros']

    if len(hosts) == 1:
        print_info('Host: ' + hosts[0][1])
    else:
        print_info('Hosts: ' + ', '.join([h[1] for h in hosts]))

    print_info('Workspace: ' + workspace)
    print()

    if not args.compile_only:
        for host in hosts:
            if args.clean_all or args.clean_src:
                clean_src_dir(host, workspace)

            print_info('Synchronizing files on {}...'.format(host[1]))
            if args.robot == 'wolfgang':
                # Filename is hostname without number (like sync_includes_wolfgang_odroid.yaml)
                sync_includes_file = 'sync_includes_wolfgang_{}.yaml'.format(host[1][:-1])
            elif args.robot == 'davros':
                sync_includes_file = 'sync_includes_davros.yaml'
            if args.package:
                synchronize(sync_includes_file, host, workspace, args.package)
            else:
                synchronize(sync_includes_file, host, workspace)

            if host[1].startswith('odroid') or host[1].startswith('nuc'):
                add_game_controller_config(host[1][-1:], workspace, host)

        print_success('Sync succeeded!')

    if not args.sync_only:
        for host in hosts:
            print_info('Compiling on {}...'.format(host[1]))
            data = dict()
            data['workspace'] = workspace
            data['jobs'] = args.jobs
            data['clean_option'] = 'catkin clean -y' if args.clean_build or args.clean_all else ''
            data['quiet_option'] = '> /dev/null' if args.quiet else ''
            data['package'] = args.package if args.package else ''

            build_result = subprocess.run([
                'ssh',
                'bitbots@{}'.format(host[0]),
                '''sync;
                cd {workspace};
                {clean_option}
                if [[ -f devel/setup.zsh ]]; then
                source devel/setup.zsh;
                catkin build --force-color -j {jobs} {package} {quiet_option} || exit 1;
                else;
                source /opt/ros/*/setup.zsh;
                catkin build --force-color -j {jobs} {package} {quiet_option} || exit 1;
                source devel/setup.zsh;
                fi;
                src/scripts/repair.sh {quiet_option};
                sync;'''.format(**data)
            ])

            if build_result.returncode != 0:
                if not args.package:
                    print_err('Build failed!')
                    exit(build_result.returncode)
                else:
                    print_warn('Build failed (or package {} does not exist on {}). Please check the build output!'.format(args.package, host[1]))
        print_success('Build succeeded!')

    if not args.compile_only and not args.sync_only and args.configure:
        configure(args)
    else:
        print_warn('Not configuring robot. Call with -k to change boot-config and game-settings')
