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
            if (-e $location . "/devel" && -d $location . "/devel"
                && -e $location . "/devel/setup.sh") {
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
        } else {
            print "Not gonna do anything then...$/";
        }
    }
}

sub create_catkin_workspace() {
    my $location = get_location();

    print "Creating new Workspace at $location$/";
    mkdir $location;
    chdir $location;
    mkdir $location . "/src";
    system "bash -c \"source $ROS_WORKSPACE/setup.bash && catkin init && catkin build\"";
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
    if (! -e "/etc/apt/sources.list.d/ros-final.list") {
        system "echo \"deb http://packages.ros.org/ros/ubuntu \$(lsb_release -sc) main\" | sudo tee /etc/apt/sources.list.d/ros-final.list"
            and die "Unable to add ROS sources";
    }
    system "sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116"
        or system "sudo apt update"
        or system "sudo apt install -y python-catkin-tools python-rosdep"
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
            or system "bash -c \"source $location/devel/setup.bash && rosdep install -i -r -y --from-paths $location/src\""
            and die "Rosdep initialization failed";
    }
}

sub install_pip() {
    # Formatting: ("package1", "package2")
    my @python2 = ();
    my @python3 = ();
    system "pip2 install --user " . join(" ", @python2) and die "pip2 package installation failed";
    system "pip3 install --user " . join(" ", @python3) and die "pip3 package installation failed";
}

sub first_catkin_build() {
    my $location = get_location();
    chdir $location;

    print "Do you want to execute 'catkin build' now?$/";
    print "[Y/n]: ";
    if (read_yes_no_input()) {
        system "bash -c \"source $location/devel/setup.bash && catkin build\"";
    }
}


prepare_ros();
prepare_workspace_location();
create_catkin_workspace();
link_bitbots_meta();
install_rosdeps();
install_pip();
first_catkin_build();
chdir $start_dir;
