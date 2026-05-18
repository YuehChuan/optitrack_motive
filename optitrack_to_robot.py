#!/usr/bin/env python3
from optitrack_vendor.NatNetClient import NatNetClient
from optitrack_vendor import DataDescriptions,MoCapData
import threading
import time
def rigid_body_callback(rb_id, pos, rot):
    x, y, z = pos
    qx, qy, qz, qw = rot
    print(f"ID:{rb_id} " f"x:{x:.3f} y:{y:.3f} z:{z:.3f} " f"qx:{qx:.3f} qy:{qy:.3f} qz:{qz:.3f} qw:{qw:.3f}")


def main():

    client = NatNetClient()

    # Motive server IP
    client.set_server_address("192.168.40.147")

    # local machine IP
    client.set_client_address("192.168.40.18")

    # use multicast (default Motive)
    client.set_use_multicast(False)
    client.rigid_body_listener = rigid_body_callback

    print("Connecting (Unicast)...")
    client.run("d")

    while True:
        time.sleep(1)



if __name__ == "__main__":
    main()
