target_locations = {
    "cart": [1, 17.5],
    "milk": [5.5, 3.5],
    "chocolate milk": [9.5, 3.5],
    "strawberry milk": [13.5, 3.5],
    "apples": [5.5, 7.5],
    "oranges": [7.5, 7.5],
    "banana": [9.5, 7.5],
    "strawberry": [11.5, 7.5],
    "raspberry": [13.5, 7.5],
    "sausage": [5.5, 11.5],
    "steak": [7.5, 11.5],
    "chicken": [11.5, 11.5],
    "ham": [13.5, 11.5],
    "brie cheese": [5.5, 15.5],
    "swiss cheese": [7.5, 15.5],
    "cheese wheel": [9.5, 15.5],
    "garlic": [5.5, 19.5],
    "leek": [7.5, 19.5],
    "red bell pepper": [9.5, 19.5],
    "carrot": [11.5, 19.5],
    "lettuce": [13.5, 19.5],
    "avocado": [5.5, 23.5],
    "broccoli": [7.5, 23.5],
    "cucumber": [9.5, 23.5],
    "yellow bell pepper": [11.5, 23.5],
    "onion": [13.5, 23.5],
    "fresh fish": [ 17.0, 15.0],
    "checkout": [4.0, 11.5],
    "exit": [-0.5, 15.6]
}

def euclidean_distance(pos1, pos2):
    # Calculate Euclidean distance between two points
    return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)**0.5


def get_next_shopping_item(state):
    shopping_list = state['observation']['players'][0]['shopping_list']
    shopping_quants = state['observation']['players'][0]['list_quant']
    user_location = state['observation']['players'][0]['position']

    #quant_dict = dict(zip(shopping_list, shopping_quants))

    curr_min_distance = 999999999
    closest_item = None

    for item in shopping_list:
        coord = target_locations[item]
        distance = euclidean_distance(user_location, coord)
        if distance < curr_min_distance:
            curr_min_distance = distance
            closest_item = item
    
    return closest_item
    # return tuple(closest_item, quant_dict[closest_item])



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
        

    