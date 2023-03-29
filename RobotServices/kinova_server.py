import socketserver
import utilities

class KinovaUDPHandler(socketserver.BaseRequestHandler):
    """
    This class works similar to the TCP handler class, except that
    self.request consists of a pair of data and client socket, and since
    there is no connection the client address must be given explicitly
    when sending data back via sendto().
    """

    def handle(self):
        data = self.request[0].strip() # message from Isaac Sim
        self.process_data(data)
        socket = self.request[1]
        print("{} get:".format(self.client_address[0]))
        print(data)

        reponse = "message back to isaac sim"
        socket.sendto(reponse.encode('utf-8'), self.client_address)

    def process_data(self, data: str):
        """
        Process data as Kinova command to control the real robot
        data is comprised of 7(body) + 1(gripper) dimensions
        """
        pass
        # joint_positions = [float(e) for e in data.split()]
        # assert len(joint_positions) == ?, "The dof is not correct"

    def control_robot(self, robot, joint_positions):
        from kinova_control import angular_action_movement
        angular_action_movement(robot, joint_positions)




if __name__ == "__main__":
    HOST, PORT = "localhost", 9999
    with socketserver.UDPServer((HOST, PORT), KinovaUDPHandler) as server:
        server.serve_forever()