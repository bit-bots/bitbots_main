.. _configure-competition-wifi:
Configuration of competition Wi-Fi
==================================

At a competition, there will be different WiFi networks for each field.
These can be setup with our ansible playbook for the robots.
This is done by editing the `group_vars/robots.yml <https://git.mafiasi.de/Bit-Bots/ansible/src/branch/master/group_vars/robots.yml>`_ config variables e.g.:

.. code-block:: yaml

   # To configure competition wifi uncomment the lines below 
   # configure team_number,  connection_name (SSID), connection_password, ip/gateway
   # and run ansible-playbook ./playbooks/setup_robots.yml --tags competition_wifi.
   
   team_number: 6
   network_configure_competition_wifi: true
   network_competition_wifi_connections:
     - connection_name: competition_field_a_ssid
       connection_password: RoboCup2023
       ip: "192.168.0.{{ team_number }}{{ player_number }}"
       gateway: 192.168.0.1
     - connection_name: competition_field_b_ssid
       connection_password: RoboCup2023
       ip: "192.168.0.{{ team_number }}{{ player_number }}"
       gateway: 192.168.0.1

Then run ``ansible-playbook ./playbooks/setup_robots.yml --tags competition_wifi`` to apply this configuration.
