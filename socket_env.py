# Author: Daniel Kasenberg (adapted from Gyan Tatiya's Minecraft socket)
import argparse
import datetime
import json
import selectors
import socket
import types

from env import SupermarketEnv, SinglePlayerSupermarketEnv
from norms.norm import NormWrapper
from norms.norms import *

import pygame

ACTION_COMMANDS = ['NOP', 'NORTH', 'SOUTH', 'EAST', 'WEST', 'INTERACT', 'TOGGLE_CART', 'CANCEL', 'SELECT','RESET']

GAME_COMMANDS = ['ESCAPE', 'SAVE', 'TOGGLE_RECORD', 'PAUSE', 'REVERT']

def serialize_data(data):
    if isinstance(data, set):
        return list(data)
    elif isinstance(data, dict):
        return {k: serialize_data(v) for k, v in data.items()}
    elif isinstance(data, list):
        return [serialize_data(item) for item in data]
    else:
        return data

class SupermarketEventHandler:
    def __init__(self, env, keyboard_input=False):
        self.env = env
        self.keyboard_input = keyboard_input
        env.reset()
        self.curr_player = env.unwrapped.game.curr_player
        self.running = True

    def single_player_action(self, action, arg=0):
        return self.curr_player, action, arg

    def record_action_and_obs(self, action, obs, arg=0):
        # Record action history
        timestamp = datetime.datetime.now().timestamp()
        self.env.unwrapped.game.action_history.append(
            (str(self.curr_player), 
             str(action).split(".")[-1], 
             str(arg), str(timestamp)))
            
        # Record observation
        self.env.unwrapped.game.save_state(obs=(str(timestamp), obs))
    
        return str(timestamp)
    
    def handle_events(self):
        if self.env.unwrapped.game.players[self.curr_player].interacting:
            self.handle_interactive_events()
        else:
            self.handle_exploratory_events()
        self.env.render(mode='violations')
        
    def reverse(self, replay_index=None):

        history_len = len(self.env.unwrapped.game.action_history)
        
        if replay_index == None: 
            replay_index = 0 if history_len < 50 else history_len - 50
        else: 
            replay_index = 0 if replay_index < 0 else replay_index

        print("replay_index: ", replay_index)
        timestamp = self.env.unwrapped.game.action_history[replay_index][-1]
        obs = self.env.unwrapped.game.game_state_observations[timestamp]
            
        self.env.unwrapped.game.set_observation(obs)
        env.render()
        self.env.unwrapped.game.update()
        # obs = self.env.unwrapped.game.observation(True)
        
        step_count = self.env.unwrapped.step_count # to preserve original step count
        
        if self.env.unwrapped.game.is_playback: 
            for action_tuple in self.env.unwrapped.game.action_history[replay_index + 1:]:
                for event in pygame.event.get():
                    if event.type == pygame.KEYDOWN and event.key == pygame.K_z:
                        self.reverse(replay_index=replay_index - 50)
                        return
                print("action_tuple[:-1]: ", action_tuple[:-1])
                action_list = list(action_tuple[:-1])
                action_list[0] = int(action_list[0])
                action_list[1] = ACTION_COMMANDS.index(action_list[1])
                self.env.step(action_list)
                
                env.render()
                pygame.time.delay(20)
                replay_index += 1
        else: 
            self.env.unwrapped.game.action_history = self.env.unwrapped.game.action_history[:replay_index + 1]
            obs_list = self.env.unwrapped.game.game_state_observations.items()
            self.env.unwrapped.game.game_state_observations = dict(filter(lambda item: (item[0] <= timestamp), obs_list))
            
        self.env.unwrapped.step_count = step_count
        
    def pause_game(self):
        waiting = True
        while waiting: 
            for event in pygame.event.get():
                if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                    self.env.unwrapped.game.running = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                    waiting = False
                elif event.type == pygame.KEYDOWN and event.key == pygame.K_z:
                    self.reverse() 

    def handle_exploratory_events(self):
        player = self.env.unwrapped.game.players[self.curr_player]
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                self.env.unwrapped.game.running = False
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_s:
                filename = input("Please enter a filename for saving the state.\n>>> ")
                self.env.unwrapped.game.save_state(filename=filename)
                print("State saved to {filename}.".format(filename=filename))
            elif self.env.unwrapped.game.is_playback:
                if event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                    self.pause_game()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_z:
                    self.reverse()
            elif event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                self.env.unwrapped.game.toggle_record()
            elif self.keyboard_input:
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN:
                        obs, _, _, _, _ = self.env.step(self.single_player_action(PlayerAction.INTERACT))
                        self.record_action_and_obs(PlayerAction.INTERACT, obs)
                    # i key shows inventory
                    elif event.key == pygame.K_i:
                        player.render_shopping_list = False
                        player.render_inventory = True
                        player.interacting = True
                    # l key shows shopping list
                    elif event.key == pygame.K_l:
                        player.render_inventory = False
                        player.render_shopping_list = True
                        player.interacting = True

                    elif event.key == pygame.K_c:
                        obs, _, _, _, _ = self.env.step(self.single_player_action(PlayerAction.TOGGLE))
                        self.record_action_and_obs(PlayerAction.TOGGLE, obs)
                    # switch players (up to 9 players)
                    else:
                        for i in range(1, len(self.env.unwrapped.game.players) + 1):
                            if i > 9:
                                continue
                            if event.key == pygame.key.key_code(str(i)):
                                self.curr_player = i - 1
                                self.env.curr_player = i - 1
                                self.env.unwrapped.game.curr_player = i - 1

                # player stands still if not moving
                elif event.type == pygame.KEYUP:
                    obs, _, _, _, _ = self.env.step(self.single_player_action(PlayerAction.NOP))
                    self.record_action_and_obs(PlayerAction.NOP, obs)

        if self.keyboard_input:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_UP]:  # up
                obs, _, _, _, _ = self.env.step(self.single_player_action(PlayerAction.NORTH))
                self.record_action_and_obs(PlayerAction.NORTH, obs)
                
            elif keys[pygame.K_DOWN]:  # down
                obs, _, _, _, _ = self.env.step(self.single_player_action(PlayerAction.SOUTH))
                self.record_action_and_obs(PlayerAction.SOUTH, obs)

            elif keys[pygame.K_LEFT]:  # left
                obs, _, _, _, _ = self.env.step(self.single_player_action(PlayerAction.WEST))
                self.record_action_and_obs(PlayerAction.WEST, obs)

            elif keys[pygame.K_RIGHT]:  # right
                obs, _, _, _, _ = self.env.step(self.single_player_action(PlayerAction.EAST))
                self.record_action_and_obs(PlayerAction.EAST, obs)

        self.running = self.env.unwrapped.game.running

    def handle_interactive_events(self):
        player = self.env.unwrapped.game.players[self.curr_player]
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE):
                self.env.unwrapped.game.running = False

            if event.type == pygame.KEYDOWN and self.keyboard_input:
                # b key cancels interaction
                if event.key == pygame.K_b:
                    obs, _, _, _, _ = self.env.step(self.single_player_action(PlayerAction.CANCEL))
                    self.record_action_and_obs(PlayerAction.CANCEL, obs)

                # return key continues interaction
                elif event.key == pygame.K_RETURN:
                    obs, _, _, _, _ = self.env.step(self.single_player_action(PlayerAction.INTERACT))
                    self.record_action_and_obs(PlayerAction.INTERACT, obs)
                # i key turns off inventory rendering
                elif event.key == pygame.K_i:
                    if player.render_inventory:
                        player.render_inventory = False
                        player.interacting = False
                # l key turns off shopping list rendering
                elif event.key == pygame.K_l:
                    if player.render_shopping_list:
                        player.render_shopping_list = False
                        player.interacting = False

                # use up and down arrows to navigate item select menu
                if self.env.unwrapped.game.item_select:
                    if event.key == pygame.K_UP:
                        self.env.unwrapped.game.select_up = True
                    elif event.key == pygame.K_DOWN:
                        self.env.unwrapped.game.select_down = True
        self.running = self.env.unwrapped.game.running


def get_action_json(action, env_, obs, reward, done, info_=None, violations=''):
    # cmd, arg = get_command_argument(action)

    if not isinstance(info_, dict):
        result = True
        message = ''
        step_cost = 0
    else:
        result, step_cost, message = info_['result'], info_['step_cost'], info_['message']

    result = 'SUCCESS' if result else 'FAIL'

    action_json = {'command_result': {'command': action, 'result': result, 'message': message,
                                      'stepCost': step_cost},
                   'observation': obs,
                   'step': env_.unwrapped.step_count,
                   'gameOver': done,
                   'violations': violations}
    # print(action_json)
    # action_json = {"hello": "world"}
    return action_json

def is_game_command(command_):
    return command_ in GAME_COMMANDS

def get_event(command_): 
    if command_ == 'ESCAPE':
        return pygame.event.Event(pygame.KEYDOWN, key=pygame.K_ESCAPE)
    elif command_ == 'SAVE':
        return pygame.event.Event(pygame.KEYDOWN, key=pygame.K_s)
    elif command_ == 'TOGGLE_RECORD':
        return pygame.event.Event(pygame.KEYDOWN, key=pygame.K_r)
    elif command_ == 'PAUSE':
        return pygame.event.Event(pygame.KEYDOWN, key=pygame.K_p)
    elif command_ == 'REVERT':
        return pygame.event.Event(pygame.KEYDOWN, key=pygame.K_z)

def is_single_player(command_):
    return ',' not in command_

def is_playback_mode(command_):
    return command == "Playback"

def get_player_and_command(command_):
    split_command = command_.split(' ')
    if len(split_command) == 1:
        return 0, split_command[0], 0
    elif len(split_command) == 2:
        return int(split_command[0]), split_command[1], 0
    return int(split_command[0]), split_command[1], int(split_command[2])


def get_commands(command_):
    split_command = [cmd.strip() for cmd in command_.split(',')]
    return split_command


def accept_wrapper(sock):
    conn, addr = sock.accept()  # Should be ready to read
    print('accepted connection from', addr)
    conn.setblocking(False)
    data = types.SimpleNamespace(addr=addr, inb=b'', outb=b'')
    events = selectors.EVENT_READ | selectors.EVENT_WRITE
    sel.register(conn, events, data=data)


if __name__ == "__main__":

    parser = argparse.ArgumentParser()

    parser.add_argument(
        '--num_players',
        type=int,
        help="location of the initial state to read in",
        default=1
    )

    parser.add_argument(
        '--port',
        type=int,
        help="Which port to bind",
        default=9000
    )

    parser.add_argument(
        '--headless',
        action='store_true'
    )

    parser.add_argument(
        '--file',
        help="location of the initial state to read in",
        default=None
    )

    parser.add_argument(
        '--follow',
        help="which agent to follow",
        type=int,
        default=-1
    )

    parser.add_argument(
        '--random_start',
        action='store_true',
    )

    parser.add_argument(
        '--keyboard_input',
        action='store_true'
    )

    parser.add_argument(
        '--render_number',
        action='store_true'
    )

    parser.add_argument(
        '--render_messages',
        action='store_true'
    )

    parser.add_argument(
        '--bagging',
        action='store_true'
    )

    parser.add_argument(
        '--player_sprites',
        nargs='+',
        type=str,
    )

    parser.add_argument(
        '--record_path',
        type=str,
    )
    
    parser.add_argument(
        '--record_actions',
        action='store_true',
    )

    parser.add_argument(
        '--stay_alive',
        action='store_true',
    )

    args = parser.parse_args()

    # np.random.seed(0)

    # Make the env
    # env_id = 'Supermarket-v0'  # NovelGridworld-v6, NovelGridworld-Pogostick-v0, NovelGridworld-Bow-v0
    # env = gym.make(env_id)
    env = SupermarketEnv(args.num_players, render_messages=args.keyboard_input, headless=args.headless,
                         initial_state_filename=args.file,
                         bagging=args.bagging,
                         follow_player=args.follow if args.num_players > 1 else 0,
                         keyboard_input=args.keyboard_input,
                         random_start=args.random_start,
                         render_number=args.render_number,
                         player_sprites=args.player_sprites,
                         record_path=args.record_path,
                         stay_alive=args.stay_alive, 
                         record_actions=args.record_actions
                        )

    norms = [CartTheftNorm(),
             BasketTheftNorm(),
             WrongShelfNorm(),
             ShopliftingNorm(),
             PlayerCollisionNorm(),
             ObjectCollisionNorm(),
             WallCollisionNorm(),
             BlockingExitNorm(),
             EntranceOnlyNorm(),
             UnattendedCartNorm(),
             UnattendedBasketNorm(),
             OneCartOnlyNorm(),
             OneBasketOnlyNorm(),
             PersonalSpaceNorm(dist_threshold=1),
             InteractionCancellationNorm(),
             LeftWithBasketNorm(),
             ReturnBasketNorm(),
             ReturnCartNorm(),
             WaitForCheckoutNorm(),
             # ItemTheftFromCartNorm(),
             # ItemTheftFromBasketNorm(),
             AdhereToListNorm(),
             TookTooManyNorm(),
             BasketItemQuantNorm(basket_max=6),
             CartItemQuantNorm(cart_min=6),
             UnattendedCheckoutNorm()
             ]

    handler = SupermarketEventHandler(NormWrapper(SinglePlayerSupermarketEnv(env), norms),
                                      keyboard_input=args.keyboard_input)
    env = NormWrapper(env, norms)
    # env.map_size = 32

    sel = selectors.DefaultSelector()

    # Connect to agent
    HOST = '127.0.0.1'
    PORT = args.port
    sock_agent = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock_agent.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock_agent.bind((HOST, PORT))
    sock_agent.listen()
    print('Listening on', (HOST, PORT))
    sock_agent.setblocking(False)

    sel.register(sock_agent, selectors.EVENT_READ, data=None)
    env.reset()
    env.render()
    done = False

    while env.unwrapped.game.running:
        events = sel.select(timeout=0) 
        should_perform_action = False
        curr_action = [(0,0)] * env.unwrapped.num_players
        e = []
        if not args.headless:
            handler.handle_events()
            env.render()
        for key, mask in events:
            if key.data is None:
                accept_wrapper(key.fileobj)
            else:
                sock = key.fileobj
                data = key.data
                if mask & selectors.EVENT_READ:
                    recv_data = sock.recv(4096)  # Should be ready to read
                    if recv_data:
                        data.inb += recv_data
                        if len(recv_data) < 4096:
                            #  We've hit the end of the input stream; now we process the input
                            command = data.inb.decode().strip()
                            data.inb = b''
                            if command.startswith("SET"):
                                obs = command[4:]
                                from json import loads
                                obs_to_return = env.reset(obs=loads(obs))
                                json_to_send = get_action_json("SET", env, obs_to_return, 0., False, None)
                                data = key.data
                                data.outb = str.encode(json.dumps(json_to_send,default=lambda o: o.__dict__) + "\n")
                            if is_playback_mode(command): 
                                env.unwrapped.game.is_playback = True
                            if is_game_command(command):
                                keydown_event = get_event(command)
                                pygame.event.post(keydown_event)
                            if is_single_player(command):
                                player, command, arg = get_player_and_command(command)
                                e.append((key, mask, command))
                                if command in ACTION_COMMANDS:
                                    action_id = ACTION_COMMANDS.index(command)
                                    curr_action[player] = (action_id, arg)
                                    should_perform_action = True
                                    # print(action)
                                else:
                                    info = {'result': False, 'step_cost': 0.0, 'message': 'Invalid Command'}
                                    json_to_send = get_action_json(command, env, None, 0., False, info, None)
                                    data.outb = str.encode(json.dumps(json_to_send) + "\n")
                    else:
                        print('closing connection to', data.addr)
                        sel.unregister(sock)
                        sock.close()
                if mask & selectors.EVENT_WRITE:
                    if data.outb:
                        sent = sock.send(data.outb)  # Should be ready to write
                        data.outb = data.outb[sent:]
        if should_perform_action:
            obs, reward, done, info, violations = env.step(tuple(curr_action))
            
            for index, player in enumerate(curr_action):
                handler.record_action_and_obs(ACTION_COMMANDS[player[0]], obs)
            
            for key, mask, command in e:
                json_to_send = get_action_json(command, env, obs, reward, done, info, violations)
                
                data = key.data
                #data.outb = str.encode(json.dumps(json_to_send) + "\n")

                # Serialize the data to ensure it's JSON-serializable
                json_to_send_serialized = serialize_data(json_to_send)                
                data.outb = str.encode(json.dumps(json_to_send_serialized) + "\n")
            env.render()
    sock_agent.close()
    if env.unwrapped.game.record_actions:
        filename = input("Please enter a filename for saving the action history.\n>>> ")
        env.unwrapped.game.write_action_history(filename)
        
            
