import socketserver
import utilities
import sys, os
from numpy import interp
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient 
from kinova_control import angular_action_movement, GripperFeedback, GripperCommand

# import files
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
# initialize connection argument

class KinovaUDPHandler(socketserver.BaseRequestHandler):
    """
    This class works similar to the TCP handler class, except that
    self.request consists of a pair of data and client socket, and since
    there is no connection the client address must be given explicitly
    when sending data back via sendto().
    """

    def handle(self):
        # obtain message from Isaac Sim
        data = self.request[0].strip()
        socket = self.request[1]
        
        print(data)
        if data.startswith(b'Hello'):
            response = "Connect with isaac sim"
            print("establish connection with isaac sim")
        else:
            joint_positions = self.process_data(data)
            success = "succeed" if self.control_robot(joint_positions) else "failed"
            response = f"The action {success}"
        socket.sendto(response.encode('utf-8'), self.client_address)

    def process_data(self, data: str):
        """
        Process data as Kinova command to control the real robot
        data is comprised of 7(body) + 1(gripper) dimensions
        """
        joint_positions = [float(e) for e in data.split()]
        return joint_positions

    def control_robot(self, joint_positions):
        from kortex_api.Exceptions.KServerException import KServerException
        with utilities.DeviceConnection.createTcpConnection(args) as router:
            with utilities.DeviceConnection.createUdpConnection(args) as router_real_time:
                base = BaseClient(router)
                base_cyclic = BaseCyclicClient(router_real_time)
                # gripper = GripperFeedback(base, base_cyclic)
                success = True
                success &= angular_action_movement(base, joint_positions[:7])
                # gripper.Cleanup()


                print("go to position", joint_positions[7])
                success &= GripperCommand(base, min(max(0, joint_positions[7]), 1))
                # gripper.Cleanup()

                return success


if __name__ == "__main__":
    HOST, PORT = "localhost", 9999
    args = utilities.parseConnectionArguments()
    with socketserver.UDPServer((HOST, PORT), KinovaUDPHandler) as server:
        server.serve_forever()
