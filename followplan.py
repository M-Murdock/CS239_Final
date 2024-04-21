

def GetCurrentState(sock_game):
    # get the current state of the environment
    output = recv_socket_data(sock_game)  # get observation from env
    output = json.loads(output)
    return state


def ExecutePlanToItem(preliminarypath): 
    # the path is a dictionary of states and actions
    # get the current state from the environment
    # find out if the current state is any of the states in the path
    # if so, execute the action
    # if not, return an error

    while len(preliminarypath) > 0:
        state = GetCurrentState()
        for key in preliminarypath:
            if state == key:
                action = preliminarypath[key]
                print("Sending action: ", action)
                sock_game.send(str.encode(action))  # send action to env
                preliminarypath.pop(key)
                break
            else:   
                return "Error: state not in path, need to replan"
        

    