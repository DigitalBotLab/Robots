import socketserver
import utilities
import sys, os
from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient


# import files
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

class KinovaUDPHandler(socketserver.BaseRequestHandler):
    """
    This class works similar to the TCP handler class, except that
    self.request consists of a pair of data and client socket, and since
    there is no connection the client address must be given explicitly
    when sending data back via sendto().
    """

    def setup(self):
        # connect to robot with default username and password
        args = utilities.parseConnectionArguments()
        self.router = utilities.DeviceConnection.createTcpConnection(args).__enter__()
        self.robot = BaseClient(self.router)


    def handle(self):
        # obtain message from Isaac Sim
        data = self.request[0].strip()
        joint_positions = self.process_data(data)
        socket = self.request[1]

        success = "succeed" if self.control_robot(joint_positions) else "failed"

        reponse = f"The action {success}"
        socket.sendto(reponse.encode('utf-8'), self.client_address)

    def process_data(self, data: str):
        """
        Process data as Kinova command to control the real robot
        data is comprised of 7(body) + 1(gripper) dimensions
        """
        pass
        joint_positions = [float(e) for e in data.split()]
        return joint_positions

    def control_robot(self, joint_positions):
        from kinova_control import angular_action_movement
        success = True
        success &= angular_action_movement(self.robot, joint_positions[:7])
        return success


if __name__ == "__main__":
    HOST, PORT = "localhost", 9999
    with socketserver.UDPServer((HOST, PORT), KinovaUDPHandler) as server:
        server.serve_forever()


        