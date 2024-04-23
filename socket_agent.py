# Author: Gyan Tatiya
# Email: Gyan.Tatiya@tufts.edu

import json
import random
import socket

from env import SupermarketEnv
from utils import recv_socket_data

def GetCurrentState(sock_game, playernumber):
    # get the current state of the environment
    smallstate = ""
    output = recv_socket_data(sock_game)  # get observation from env
    state = json.loads(output)
    for key in state["observation"]: 
     for key2 in state["observation"][key]: 
        if key == "players":
            if key2['index'] == playernumber:
                currentposition = key2["position"]
                print("currentposition: ", currentposition)
                carts = key2["curr_cart"]
    smallstate = str(carts)+str(currentposition)
    print("smallstate: ", smallstate)
    print(state)
    return smallstate

if __name__ == "__main__":

    # Make the env
    # env_id = 'Supermarket-v0'
    # env = gym.make(env_id)

    action_commands = ['NOP', 'NORTH', 'SOUTH', 'EAST', 'WEST', 'TOGGLE_CART', 'INTERACT']

    print("action_commands: ", action_commands)

    # Connect to Supermarket
    HOST = '127.0.0.1'
    PORT = 9000
    sock_game = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_game.connect((HOST, PORT))

    while True:
        # action = str(random.randint(0, 1))
        # action += " " + random.choice(action_commands)  # random action

        # assume this is the only agent in the game
        action = "0 " + "SOUTH"

        print("Sending action: ", action)
        sock_game.send(str.encode(action))  # send action to env

        #output = recv_socket_data(sock_game)  # get observation from env
        #output = json.loads(output)

        state = GetCurrentState(sock_game, 0)

        print("State: ", state)
        #print("Observations: ", output["observation"])
        #print("Violations", output["violations"])
