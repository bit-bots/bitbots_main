#!/usr/bin/perl -w
use strict;
use warnings;

use Cwd;
use File::Path;

my $ROS_WORKSPACE = "/opt/ros/melodic";
my $start_dir = Cwd::cwd();

sub get_location() {
    my $location = $ENV{'HOME'} . '/catkin_ws';
    if (defined $ENV{'CATKIN_WS'}) {
        $location = $ENV{'CATKIN_WS'};
    }
    return $location;
}

sub read_yes_no_input() {
    # Interpret input
    chomp(my $input = <STDIN>);
    $input = lc $input;
    if ($input eq 'yes' || $input eq 'y' || $input eq "") {
        return 1;
    }
    else {
        return 0;
    }
}


sub prepare_workspace_location() {
    my $location = get_location();

    if (-e $location) {
        # Ask the user how to handle the existing entity at catkin_ws location
        if (-d $location) {
            if (-e $location . "/.catkin_tools" && -d $location . "/.catkin_tools") {
                print "$location seems to already be a Catkin Workspace.$/";
                print "Do you want me to remove it and create a new one?$/";
                print "[Y/n]: ";
            }
            else {
                print "Directory $location already exists but doesn't seem to be a Catkin Workspace.$/";
                print "Do you want me to remove it and create a new Catkin Workspace there?$/";
                print "[Y/n]: ";
            }
        }
        else {
            print "$location already exists but is not a directory.$/";
            print "Do you want me to remove the existing file and create a Catkin Workspace there?$/";
            print "[Y/n]: ";
        }

        # Remove the existing location if answer is yes
        if (read_yes_no_input()) {
            print "Ok. Deleting old location$/";
            rmtree $location;
            unlink $location;
            print "Creating new Workspace at $location$/";
            mkdir $location;
            mkdir $location . "/src";
        } else {
            print "Not gonna delete it then...$/";
        }
    } else {
        print "Creating new Workspace at $location$/";
        mkdir $location;
        mkdir $location . "/src";
    }
}

sub configure_catkin_workspace() {
    my $location = get_location();

    print "Configuring workspace at $location$/";
    chdir $location;
    # human_pose_estimation is currently blacklisted because it doesn't build
    system "bash -c \"source $ROS_WORKSPACE/setup.bash && catkin config --init -DPYTHON_VERSION=3 --blacklist human_pose_estimation_openvino\"";
}

sub link_bitbots_meta() {
    my $location = get_location();

    print "I will assume bitbots_meta to be at you current location ($start_dir).$/";
    print "Is that correct? [Y/n] ";

    if (read_yes_no_input()) {
        print "Linking bitbots_meta$/";
        system "ln -sT $start_dir $location/src/bitbots_meta";
    } else {
        die "Please navigate to bitbots_meta then...$/";
    }
}

sub prepare_ros() {
    print "Rosdep will be installed to resolve ROS dependecies$/";
    if (-e "/etc/apt/sources.list.d/ros-latest.list") {
        system "sudo rm /etc/apt/sources.list.d/ros-latest.list"
            and die "Unable to remove old ros source list";
    }
    if (-e "/etc/apt/sources.list.d/ros-final.list") {
        system "sudo rm /etc/apt/sources.list.d/ros-final.list"
            and die "Unable to remove old ros source list";
    }
    if (! -e "/etc/apt/sources.list.d/ros.list") {
        system "sudo sh -c 'echo \"deb http://packages.bit-bots.de bionic main\" > /etc/apt/sources.list.d/ros.list'"
            and die "Unable to add ROS sources";
    }
    system "sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key 4C4EDF893374591687621C75C2F8DBB6A37B2874"
        or system "sudo apt update"
        or system "sudo apt install -y python3-pip python3-rosdep"
    or system "sudo pip3 install git+https://github.com/catkin/catkin_tools.git"
        and die "Unable to install rosdep";
    system "sudo apt install -y ros-melodic-desktop-full"
        and die "Unable to install ROS";
}

sub install_rosdeps() {
    my $location = get_location();

    print "Do you want to use rosdep to resolve dependencies now?$/";
    print "[Y/n]: ";
    if (read_yes_no_input()) {
        system "sudo rosdep init";
        system "rosdep update"
            or system "bash -c \"source /opt/ros/melodic/setup.bash && rosdep install -i -r -y --from-paths $location/src\""
            and die "Rosdep initialization failed";
    }
}

sub install_pip() {
    print "Installing python dependencies...$/";
    # Formatting: ("package1", "package2")
    my @python3 = ("defusedxml");
    system "pip3 install --user " . join(" ", @python3) and die "pip3 package installation failed";
}

sub first_catkin_build() {
    my $location = get_location();
    chdir $location;

    print "Do you want to execute 'catkin build' now?$/";
    print "[Y/n]: ";
    if (read_yes_no_input()) {
        system "bash -c \"source /opt/ros/melodic/setup.bash && catkin build\"";
    }
}

sub clone_internal_repos() {
    # Clone internal bitbots_repos into bitbots_meta
    chdir $start_dir;

    print "Do you want to clone the private repos ('ansible', 'doku' and 'basler_drivers') now?$/";
    print "[Y/n]: ";
    if (read_yes_no_input()) {
        if (! -e "ansible") {
            system 'git clone git@git.mafiasi.de:Bit-Bots/ansible.git bitbots_tools/ansible'
                and print "Could not clone ansible_robots$/$/";
        } else {
            print "ansible is already cloned$/";
        }

        if (! -e "doku") {
            system 'git clone git@git.mafiasi.de:Bit-Bots/doku.git bitbots_tools/bitbots_docs_internal'
                and print "Could not clone doku$/$/";
        } else {
            print "doku is already cloned$/";
        }

        if (! -e "basler_drivers") {
            system 'git clone git@git.mafiasi.de:Bit-Bots/basler_drivers.git'
                and print "Could not clone basler_drivers$/$/";
        } else {
            print "basler_drivers is already cloned$/";
        }
        print "Installing basler drivers...$/";
        system 'make basler';
    }
}


prepare_ros();
prepare_workspace_location();
configure_catkin_workspace();
link_bitbots_meta();
install_rosdeps();
install_pip();
clone_internal_repos();
first_catkin_build();
chdir $start_dir;
print "Congratulations, the installation was successful!$/";
