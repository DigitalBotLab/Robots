import socketserver

class KinovaUDPHandler(socketserver.BaseRequestHandler):
    """
    This class works similar to the TCP handler class, except that
    self.request consists of a pair of data and client socket, and since
    there is no connection the client address must be given explicitly
    when sending data back via sendto().
    """

    def handle(self):
        data = self.request[0].strip()
        self.process_data(data)
        socket = self.request[1]
        print("{} wrote:".format(self.client_address[0]))
        print(data)
        socket.sendto(data.upper(), self.client_address)

    def process_data(self, data: str):
        """
        Process data as Kinova command to control the real robot
        data is comprised of 7(body) + 6(gripper) dimensions
        """
        pass
        # joint_positions = [float(e) for e in data.split()]
        # assert len(joint_positions) == ?, "The dof is not correct"

    def control_robot(self, joint_positions):
        pass



if __name__ == "__main__":
    HOST, PORT = "localhost", 9999
    with socketserver.UDPServer((HOST, PORT), KinovaUDPHandler) as server:
        server.serve_forever()