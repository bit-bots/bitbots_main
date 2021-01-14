Time Synchronization
====================
Because our robots rely on having accurate information about when data was generated, they need to have their clocks synchronized.
This document aims to explain how such synchronization is achieved.

Source-of-truth
---------------
All our robots are configured via ansible so for a complete reference check the roles
`timesync_master <https://git.mafiasi.de/Bit-Bots/ansible/src/branch/master/roles/timesync_master>`_ and
`timesync_slave <https://git.mafiasi.de/Bit-Bots/ansible/src/branch/master/roles/timesync_slave>`_.
The configuration directives are explained via comments in their respective config files (see the template folder).

In summary, we use chrony on our nuc as a time server.
It uses *Universität Hamburg* as well as debian for upstream synchronization.
All other robot computers on the same robot then use the nuc as time servers.

.. warning::
    We don't currently synchronize time between robots.

    While it should be possible to do so via the chrony *peer mode*, we do not yet have it configured.
