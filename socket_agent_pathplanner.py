import json
import random
import socket

from env import SupermarketEnv
from utils import recv_socket_data
import followplan   
import preliminary_path


HOST = '127.0.0.1'
PORT = 9000
sock_game = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock_game.connect((HOST, PORT))
sock_game.send(str.encode("0 RESET"))  # reset the game

print("Which player number are you?")
playernumber = int(input())
state = recv_socket_data(sock_game)
game_state = json.loads(state)
print("got state for the first time")

shopping_list = game_state['observation']['players'][playernumber]['shopping_list']
shopping_quant = game_state['observation']['players'][playernumber]['list_quant']
player = preliminary_path.PathPlanner()
print("Created Preliminary Path")
itemcount = len(shopping_list)

if itemcount > 6:
    print("More than 6")
    path = player.grab_cart_or_basket(game_state, kind="cart")
    print("got the path to cart")
    followplan.ExecutePlanToItem(path, sock_game, playernumber)
    has_cart = True
else:
    print("less than 6")
    path = player.grab_cart_or_basket(game_state, kind="basket")
    print("got the path to basket")
    followplan.ExecutePlanToItem(path, sock_game, playernumber)
    has_cart = False

while len(shopping_list) > 0:
    nextitem = followplan.get_next_shopping_item(state)
    path = player.get_path_to_item(game_state, nextitem, has_cart)
    followplan.ExecutePlanToItem(path, sock_game, playernumber)
    
path = player.checkout(game_state)
followplan.ExecutePlanToItem(path, sock_game)


sock_game.close()
     
