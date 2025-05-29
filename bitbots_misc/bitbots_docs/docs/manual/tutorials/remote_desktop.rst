=====================
Remote Desktop Access
=====================

Since the upgrade to Ubuntu 24.04, which is using gnome-shell >= 46, it supports integrated remote desktop access over RDP.  
But because the cls are not directly accessible via the internet, we need to use a tunnel to access them.
Until we have a better solution, we can just use an SSH tunnel over the university jump hosts `{rzssh1, rzssh2}.informatik.uni-hamburg.de`.

So using the normal external SSH access `uni-jump-host -> rzrobo3 -> cl0*` we can create a tunnel on a specific port, which we can then use to connect to the remote desktop:

.. code-block:: bash
    ssh -CL 3389:cl06:3389 rzrobo3 -N 

This will forward the RDP port 3389 from the remote desktop of `cl06` to your local machine's port 3389, 
then allowing you to connect to it using an RDP client like `Remmina` or `Microsoft Remote Desktop` with `127.0.0.1:3389` as the address.
**This assumes that you have an SSH config set up for `rzrobo3`, which uses the university jump hosts.**

The remote desktop login is using the user `bitbots` and the password is located in our vault under "Gnome RDP Login".
