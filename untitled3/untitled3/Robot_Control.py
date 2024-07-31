import sys
import math
import time
import queue
import datetime
import random
import traceback
import threading
from xarm import version
from xarm.wrapper import XArmAPI
from threading import Thread, Event
import socket
import json
import rclpy as rp
from rclpy.node import Node
from untitled_msgs.msg import TopicString
from untitled_msgs.srv import ServiceString
from untitled_msgs.action import ActionString
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# 모듈이 위치한 경로를 추가합니다
xarm_path = '/home/jchj/Untitled3/src/untitled3/xarm'
sys.path.append(xarm_path)

class RobotMain(object):
    """Robot Main Class"""

    def __init__(self, robot, **kwargs):
        self.alive = True
        self._arm = robot
        self._tcp_speed = 100
        self._tcp_acc = 2000
        self._angle_speed = 20
        self._angle_acc = 500
        self._tcp_speed_gripper = 100
        self._angle_speed_gripper = 100
        self._vars = {}
        self._funcs = {}
        self._robot_init()
        self.state = 'stopped'
        self.stop_requested = threading.Event()

        self.position_home = [173.7, -25.9, 7.1, 179.8, 59, -1.6] #angle
        # home_position = [-170, 18.6, 281.9, -52.1, 89.1, 126.1] # linear
        self.position_jig_A_grab = [-257.3, -138.3, 198, 68.3, 86.1, -47.0] #linear
        self.position_jig_B_grab = [-152.3, -129.0, 198, 4.8, 89.0, -90.7] #linear
        self.position_jig_C_grab = [-76.6, -144.6, 198, 5.7, 88.9, -50.1] #linear
        self.position_sealing_check = [-136.8, 71.5, 307.6, 69.6, -73.9, -59] #Linear
        self.position_capsule_place = [234.9, 135.9, 465.9, 133.6, 87.2, -142.1] #Linear
        self.position_before_capsule_place = self.position_capsule_place.copy()
        self.position_before_capsule_place[2] += 25
        self.position_cup_grab = [214.0, -100.2, 145.0, -25.6, -88.5, 95.8] #linear
        self.position_topping_A = [-200.3, 162.8, 359.9, -31.7, 87.8, 96.1] #Linear
        self.position_topping_B = [106.5, -39.7, 15.0, 158.7, 40.4, 16.9] #Angle
        self.position_topping_C = [43.6, 137.9, 350.1, -92.8, 87.5, 5.3] #Linear
        self.position_icecream_with_topping = [168.7, 175.6, 359.5, 43.9, 88.3, 83.3] #Linear
        self.position_icecream_no_topping = [48.4, -13.8, 36.3, 193.6, 42.0, -9.2] #angle
        self.position_jig_A_serve = [-258.7, -136.4, 208.2, 43.4, 88.7, -72.2] #Linear
        self.position_jig_B_serve = [-166.8, -126.5, 200.9, -45.2, 89.2, -133.6] #Linear
        self.position_jig_C_serve = [-63.1, -138.2, 199.5, -45.5, 88.1, -112.1] #Linear
        self.position_capsule_grab = [234.2, 129.8, 464.5, -153.7, 87.3, -68.7] #Linear

        # Robot init
    def _robot_init(self):
        self._arm.clean_warn()
        self._arm.clean_error()
        self._arm.motion_enable(True)
        self._arm.set_mode(0)
        self._arm.set_state(0)
        time.sleep(1)
        self._arm.register_error_warn_changed_callback(self._error_warn_changed_callback)
        self._arm.register_state_changed_callback(self._state_changed_callback)
        if hasattr(self._arm, 'register_count_changed_callback'):
            self._arm.register_count_changed_callback(self._count_changed_callback)

    # Register error/warn changed callback
    def _error_warn_changed_callback(self, data):
        if data and data['error_code'] != 0:
            self.alive = False
            self.pprint('err={}, quit'.format(data['error_code']))
            self._arm.release_error_warn_changed_callback(self._error_warn_changed_callback)

    # Register state changed callback
    def _state_changed_callback(self, data):
        if data and data['state'] == 4:
            self.alive = False
            self.pprint('state=4, quit')
            self._arm.release_state_changed_callback(self._state_changed_callback)

    # Register count changed callback
    def _count_changed_callback(self, data):
        if self.is_alive:
            self.pprint('counter val: {}'.format(data['count']))

    def _check_code(self, code, label):
        if not self.is_alive or code != 0:
            self.alive = False
            ret1 = self._arm.get_state()
            ret2 = self._arm.get_err_warn_code()
            self.pprint('{}, code={}, connected={}, state={}, error={}, ret1={}. ret2={}'.format(label, code,
                                                                                                 self._arm.connected,
                                                                                                 self._arm.state,
                                                                                                 self._arm.error_code,
                                                                                                 ret1, ret2))
        return self.is_alive

    @staticmethod
    def pprint(*args, **kwargs):
        try:
            stack_tuple = traceback.extract_stack(limit=2)[0]
            print('[{}][{}] {}'.format(time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(time.time())), stack_tuple[1],
                                       ' '.join(map(str, args))))
        except:
            print(*args, **kwargs)

    @property
    def arm(self):
        return self._arm

    @property
    def VARS(self):
        return self._vars

    @property
    def FUNCS(self):
        return self._funcs

    @property
    def is_alive(self):
        if self.alive and self._arm.connected and self._arm.error_code == 0:
            if self._arm.state == 5:
                cnt = 0
                while self._arm.state == 5 and cnt < 5:
                    cnt += 1
                    time.sleep(0.1)
            return self._arm.state < 4
        else:
            return False

    def position_reverse_sealing_fail(self, linear_jig_position = [-257.3, -138.3, 192.1, 68.3, 86.1, -47.0]):
        reverse_position = linear_jig_position.copy()
        reverse_position[2] = reverse_position[2] - 10
        reverse_position[3] = -reverse_position[3]
        reverse_position[4] = -reverse_position[4]
        reverse_position[5] = reverse_position[5] - 180
        return reverse_position

    def socket_connect(self):
        self.HOST = '192.168.1.192'
        self.PORT = 20002
        self.BUFSIZE = 1024
        self.ADDR = (self.HOST, self.PORT)

        try:
            self.clientSocket.shutdown(1)
            self.clientSocket.close()
        except:
            pass

        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        while True:
            try:
                self.serverSocket.bind(self.ADDR)
                print("bind")

                while True:
                    self.serverSocket.listen(1)
                    print(f'[LISTENING] Server is listening on robot_server')
                    time.sleep(1)
                    try:
                        while True:
                            try:
                                self.clientSocket, addr_info = self.serverSocket.accept()
                                print("socket accepted")
                                break
                            except:
                                time.sleep(1)
                                print('except')
                                # break

                        break

                    except socket.timeout:
                        print("socket timeout")

                    except:
                        pass
                break
            except:
                pass
        print("accept")
        print("--client info--")

        self.connected = True
        self.state = 'ready'

        # ------------------- receive msg start -----------
        while self.connected:
            print('loop start')
            time.sleep(0.5)
            try:
                print('waiting')
                self.clientSocket.settimeout(10.0)
                self.recv_msg = self.clientSocket.recv(1024).decode('utf-8')
                print('\n' + self.recv_msg)
                if self.recv_msg == '':
                    print('here')
                    raise Exception('empty msg')
                self.recv_msg = self.recv_msg.split('/')

                if self.recv_msg[0] == 'app_ping':
                    send_msg = 'robot_ping'
                    now_temp = self._arm.temperatures
                    now_cur = self._arm.currents
                    send_msg = [
                        {
                            'type': 'A', 'joint_name': 'Base', 'temperature': now_temp[0],
                            'current': round(now_cur[0], 3) * 100
                        }, {
                            'type': 'B', 'joint_name': 'Shoulder', 'temperature': now_temp[1],
                            'current': round(now_cur[1], 3) * 100
                        }, {
                            'type': 'C', 'joint_name': 'Elbow', 'temperature': now_temp[2],
                            'current': round(now_cur[2], 3) * 100
                        }, {
                            'type': 'D', 'joint_name': 'Wrist1', 'temperature': now_temp[3],
                            'current': round(now_cur[3], 3) * 100
                        }, {
                            'type': 'E', 'joint_name': 'Wrist2', 'temperature': now_temp[4],
                            'current': round(now_cur[4], 3) * 100
                        }, {
                            'type': 'F', 'joint_name': 'Wrist3', 'temperature': now_temp[5],
                            'current': round(now_cur[5], 3) * 100
                        }
                    ]
                    try:
                        time.sleep(0.5)
                        self.clientSocket.send(f'{send_msg}'.encode('utf-8'))
                        print('robot_ping')

                    except Exception as e:
                        self.pprint('MainException: {}'.format(e))
                        print('ping send fail')
                    # send_msg = arm.temperatures
                    if self.state == 'ready':
                        print('STATE : ready for new msg')
                    else:
                        print('STATE : now moving')
                else:
                    self.recv_msg[0] = self.recv_msg[0].replace("app_ping", "")
                    if self.recv_msg[0] in ['breath', 'greet', 'farewell' 'dance_random', 'dance_a', 'dance_b',
                                            'dance_c',
                                            'sleep', 'comeon']:
                        print(f'got message : {self.recv_msg[0]}')
                        if self.state == 'ready':
                            self.state = self.recv_msg[0]
                    elif self.recv_msg[0] == 'robot_script_stop':
                        code = self._arm.set_state(4)
                        if not self._check_code(code, 'set_state'):
                            return
                        sys.exit()
                        self.is_alive = False
                        print('program exit')

                    # 픽업존 아이스크림 뺐는지 여부 확인
                    elif self.recv_msg[0].find('icecream_go') >= 0 or self.recv_msg[0].find(
                            'icecream_stop') >= 0 and self.state == 'icecreaming':
                        print(self.recv_msg[0])
                        if self.recv_msg[0].find('icecream_go') >= 0:
                            self.order_msg['makeReq']['latency'] = 'go'
                        else:
                            self.order_msg['makeReq']['latency'] = 'stop'
                            print('000000000000000000000000000000')

                    # 실링 존재 여부 확인

                    if self.recv_msg[0].find('sealing_pass') >= 0 and self.state == 'icecreaming':
                        self.order_msg['makeReq']['sealing'] = 'go'
                        print('socket_go')
                    elif self.recv_msg[0].find('sealing_reject') >= 0 and self.state == 'icecreaming':
                        self.order_msg['makeReq']['sealing'] = 'stop'
                        print('socket_stop')

                    else:
                        try:
                            self.order_msg = json.loads(self.recv_msg[0])
                            if self.order_msg['type'] == 'ICECREAM':
                                if self.state == 'ready':
                                    print('STATE : icecreaming')
                                    print(f'Order message : {self.order_msg}')
                                    self.state = 'icecreaming'
                            else:
                                self.clientSocket.send('ERROR : wrong msg received'.encode('utf-8'))
                        except:
                            pass
                self.recv_msg[0] = 'zzz'

            except Exception as e:
                self.pprint('MainException: {}'.format(e))
                print('connection lost')
                while True:
                    time.sleep(2)
                    try:

                        try:
                            self.serverSocket.shutdown(socket.SHUT_RDWR)
                            self.serverSocket.close()
                        except:
                            pass

                        print('socket_making')
                        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        self.serverSocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                        self.serverSocket.bind(self.ADDR)
                        print("bind")

                        while True:
                            print('listening')
                            self.serverSocket.listen(1)
                            print(f'reconnecting')
                            try:
                                self.clientSocket, addr_info = self.serverSocket.accept()
                                break

                            except socket.timeout:
                                print('socket.timeout')
                                break

                            except:
                                pass
                        break
                    except Exception as e:
                        self.pprint('MainException: {}'.format(e))
                        print('except')
                        # pass

    # =================================  motion  =======================================
    def motion_home(self):

        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return

        # press_up
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return

        # Joint Motion
        self._angle_speed = 80
        self._angle_acc = 200
        try:
            self.clientSocket.send('motion_home_start'.encode('utf-8'))
        except:
            print('socket error')
        print('motion_home start')
        # designed home
        code = self._arm.set_servo_angle(angle=self.position_home, speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        print('motion_home finish')

    def motion_grab_capsule(self , params):

        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        
        # self.order_msg['makeReq']['jigNum'] = params으로 대체

        # Joint Motion
        self._angle_speed = 100
        self._angle_acc = 100

        self._tcp_speed = 100
        self._tcp_acc = 1000

        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)
        
        if params in ['A']:
            pass
        else:
            code = self._arm.set_servo_angle(angle=[176, 31.7, 31, 76.7, 91.2, -1.9], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)

        if params == 'A':
            code = self._arm.set_servo_angle(angle=[179.5, 33.5, 32.7, 113.0, 93.1, -2.3], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*self.position_jig_A_grab, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif params == 'B':

            code = self._arm.set_position(*self.position_jig_B_grab, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif params == 'C':
            code = self._arm.set_servo_angle(angle=[182.6, 27.8, 27.7, 55.7, 90.4, -6.4], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*self.position_jig_C_grab, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(1)
        if params == 'C':
            code = self._arm.set_position(z=150, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                          wait=False)
            if not self._check_code(code, 'set_position'):
                return
            self._tcp_speed = 200
            self._tcp_acc = 1000
            code = self._arm.set_tool_position(*[0.0, 0.0, -90.0, 0.0, 0.0, 0.0], speed=self._tcp_speed,
                                               mvacc=self._tcp_acc, wait=False)
            if not self._check_code(code, 'set_position'):
                return
        else:
            code = self._arm.set_position(z=100, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                          wait=False)
            if not self._check_code(code, 'set_position'):
                return

        self._angle_speed = 180
        self._angle_acc = 500

    def sealing_check(self,params):
        if params == 'A' or 'B' or 'C':
            code = self._arm.set_servo_angle(angle=[145, -18.6, 10.5, 97.5, 81.4, 145], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        else :
            code = self._arm.set_servo_angle(angle=[146.1, -10.7, 10.9, 102.7, 92.4, 24.9], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        print('sealing check')
        self._angle_speed = 200
        self._angle_acc = 200
        code = self._arm.set_position(*self.position_sealing_check, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return

    def motion_place_fail_capsule(self,params):
        if params == 'A':
            code = self._arm.set_servo_angle(angle=[177.3, 5.5, 12.9, 133.6, 81.3, 183.5], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*self.position_reverse_sealing_fail(self.position_jig_A_grab), speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif params == 'B':
            code = self._arm.set_servo_angle(angle=[159.5, 11.8, 22.2, 75.6, 92.8, 186.6], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*self.position_reverse_sealing_fail(self.position_jig_B_grab) , speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif params == 'C':
            code = self._arm.set_servo_angle(angle=[176.9, -2.2, 15.3, 69.3, 87.5, 195.5], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*self.position_reverse_sealing_fail(self.position_jig_C_grab) , speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)

        code = self._arm.set_position(z=100, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                      wait=False)
        if not self._check_code(code, 'set_position'):
            return

    def motion_dance_c(self):  # designed '빙글빙글'
        try:
            self.clientSocket.send('dance_c_start'.encode('utf-8'))
        except:
            print('socket error')

        self._angle_speed = 150
        self._angle_acc = 700
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(3)):
            if not self.is_alive:
                break
            t1 = time.monotonic()
            code = self._arm.set_servo_angle(angle=[180.0, 70.0, 250.0, 173.1, 0.0, -135.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, -70.0, 110.0, 180.0, 0.0, 135.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            interval = time.monotonic() - t1
            if interval < 0.01:
                time.sleep(0.01 - interval)
        code = self._arm.set_servo_angle(angle=[180.0, 70.0, 250.0, 173.1, 0.0, -135.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        while True:
            try:
                self.clientSocket.send('dance_c_finish'.encode('utf-8'))
                break
            except:
                print('socket error')

    def motion_come_on(self):  # designed '컴온컴온

        try:
            self.clientSocket.send('comeon_start'.encode('utf-8'))
        except:
            print('socket error')
        self._angle_speed = 80
        self._angle_acc = 400
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[180.0, 70.0, 220.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        for i in range(int(2)):
            if not self.is_alive:
                break
            t1 = time.monotonic()

            print(self.stop_requested)
            if self.stop_requested.is_set():
                break
            
            code = self._arm.set_servo_angle(angle=[180.0, 70.0, 220.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 62.0, 222.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 55.0, 222.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 45.0, 222.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 35.0, 224.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 25.0, 224.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 15.0, 226.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 5.0, 226.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 0.0, 228.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 5.0, 230.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 20.0, 226.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 35.0, 226.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 45.0, 228.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 55.0, 226.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 65.0, 224.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_servo_angle(angle=[180.0, 70.0, 222.0, 90.0, 20.0, 0.0], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=False, radius=30.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            interval = time.monotonic() - t1
            if interval < 0.01:
                time.sleep(0.01 - interval)
        code = self._arm.set_servo_angle(angle=[180.0, 65.0, 222.0, 90.0, 60.0, 0.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        while True:
            try:
                self.clientSocket.send('comeon_finish'.encode('utf-8'))
                break
            except:
                print('socket error')

    def motion_greet(self):
        try:
            self.clientSocket.send('greet_start'.encode('utf-8'))
        except:
            print('socket error')

        self._angle_speed = 100
        self._angle_acc = 350

        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 181.5, -1.9, -92.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 180.9, -28.3, -92.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 185.4, 30.8, -94.9], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 180.9, -28.3, -92.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        # code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 185.4, 30.8, -94.9], speed=self._angle_speed,
        #                                  mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        # code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 180.9, -28.3, -92.8], speed=self._angle_speed,
        #                                  mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 185.4, 30.8, -94.9], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        try:
            self.clientSocket.send('motion_greet finish'.encode('utf-8'))
        except:
            print('socket error')
        code = self._arm.set_servo_angle(angle=[178.9, -0.7, 179.9, 181.5, -1.9, -92.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        while True:
            try:
                self.clientSocket.send('motion_greet_finish'.encode('utf-8'))
                break
            except:
                print('socket error')

    def joint_state(self):
        while self.is_alive:
            print(f'joint temperature : {self._arm.temperatures}')
            time.sleep(0.5)
            print(f'joint current : {self._arm.currents}')
            time.sleep(10)
           
    def trash(self, aruco_x, aruco_y):
        aruco_y_1 = 25
        aruco_y_2 = 25
        aruco_y_3 = 0
        print(aruco_x,aruco_y)
        if -115 <= aruco_y <= -63: # 그리퍼 y -28 ~ 24
            aruco_y_1 = aruco_y + 87
            print(aruco_x,aruco_y_1) 
        elif 27 <= aruco_y <= 74: # 그리퍼 y -60 ~ -13
            aruco_y_2 = aruco_y - 87
            print(aruco_x,aruco_y_2)
        elif -63 < aruco_y < 27: #그리퍼 xx
            aruco_xx = aruco_x + 105
            aruco_y_3 = aruco_y 
            print(aruco_xx,aruco_y_3)
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):    # 홈 위치에서 gripper open
            return
        time.sleep(1)
        if aruco_x <= 0:
            if -60 <= aruco_y_2 <= -13:
                print(aruco_y_2,"ok1")
                code = self._arm.set_position(*[aruco_x, aruco_y_2, 463.4, -52.1, 89.1, 126.1], speed=self._tcp_speed,    # z값 높게 이동
                                        mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[aruco_x, aruco_y_2, 463.4, 43.8, -88.5, -46.2], speed=self._tcp_speed_gripper,    # gripper 180도 회전
                                            mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(2.5)
                code = self._arm.set_position(*[aruco_x, aruco_y_2, 350, 43.8, -88.5, -133.9], speed=self._tcp_speed,   # 컵이랑 충돌 방지 way point, z1 & gripper 방향 동시 이동
                                            mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_position(*[aruco_x, aruco_y_2, 153, 43.8, -88.5, -133.9], speed=self._tcp_speed,   # z1 & gripper 방향 동시 이동
                                            mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
            if -28 <= aruco_y_1 <= 24:
                print(aruco_y_1,"ok2")
                code = self._arm.set_position(*[aruco_x, aruco_y_1, 463.4, -52.1, 89.1, 126.1], speed=self._tcp_speed,    # z값 높게 이동
                                        mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[aruco_x, aruco_y_1, 463.4, 43.8, -88.5, -46.2], speed=self._tcp_speed_gripper,    # gripper 180도 회전
                                            mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(2.5)
                code = self._arm.set_position(*[aruco_x, aruco_y_1, 353, 43.8, -88.5, 46.2], speed=self._tcp_speed,  # 컵이랑 충돌 방지 way point, z1 & gripper 방향 동시 이동
                                            mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_position(*[aruco_x, aruco_y_1, 153, 43.8, -88.5, 46.2], speed=self._tcp_speed,  # z1 & gripper 방향 동시 이동
                                            mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
            if -63 < aruco_y_3 < 27:
                print("ok3")
                code = self._arm.set_position(*[aruco_xx, aruco_y_3, 463.4, -52.1, 89.1, 126.1], speed=self._tcp_speed,    # z값 높게 이동
                                        mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(1)
                code = self._arm.set_position(*[aruco_xx, aruco_y_3, 463.4, 43.8, -88.5, -46.2], speed=self._tcp_speed_gripper,    # gripper 180도 회전
                                            mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                time.sleep(2.5)
                code = self._arm.set_position(*[aruco_xx, aruco_y_3, 153, 43.8, -88.5, -46.2], speed=self._tcp_speed,    # z1 & gripper 방향 동시 이동
                                        mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
            time.sleep(2)
            code = self._arm.close_lite6_gripper()                      # gripper close
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            time.sleep(4)
            code = self._arm.set_position(-170, 18.6, 281.9, 43.8, -88.5, -46.2, speed=50, mvacc=100, wait=True, relative=False)    # 홈 위치로 복귀
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_servo_angle(angle=[171.6, -26.2, 7.2, 187.1, 21.9, 182.9], speed=self._angle_speed,   # 그리퍼 기울이기 - 회전시 충돌 방지
                                                mvacc=self._angle_acc, wait=True, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            time.sleep(1)
        elif aruco_x >= 0:
            code = self._arm.set_position(-156.8, 6.6, 225.6, -52.1, 89.1, 126.1, speed=50, mvacc=100, wait=True, relative=False)   # 회전을 위해 자세 조정
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_servo_angle(angle = [-1.6, -3.9, 1.8, 181, 84.8, -0.6], speed=self._angle_speed,        # +x 방향으로 회전
                                    mvacc=self._tcp_acc, radius=20.0, wait=True)
            if not self._check_code(code, 'set_servo_angle'):
                return
            time.sleep(1)
            code = self._arm.set_position(157, -3.3, 359.1, -54.5, 89.1, -55.1,speed=50, mvacc=100, wait=True, relative=False)  # 회전 후 자세 조정
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_position(235.2, 5.3, 359.1, -52.1, 89.1, -53.2, speed=50, mvacc=100, wait=True, relative=False) # 회전 후 자세 조정
            if not self._check_code(code, 'set_position'):
                return
            time.sleep(1)
            code = self._arm.set_servo_angle(angle=[2.1, -16.7, 34.1, 175, 39.8, 175.9], speed=self._angle_speed_gripper,    # gripper 180도 회전
                                                mvacc=self._angle_acc, wait=True, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            time.sleep(1)
            if aruco_y > 0:
                code = self._arm.set_position(*[aruco_x, aruco_y, 359.1, 43.8, -88.5, -133.9], speed=self._tcp_speed,   # 컵과 충돌 방지를 위한 way point, z1 & gripper 방향 동시 이동
                                            mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return                
                code = self._arm.set_position(*[aruco_x, aruco_y, 153, 43.8, -88.5, -133.9], speed=self._tcp_speed,   # z1 & gripper 방향 동시 이동
                                            mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
            elif aruco_y < 0:
                code = self._arm.set_position(*[aruco_x, aruco_y, 359.1, 43.8, -88.5, 46.2], speed=self._tcp_speed,     # 컵과 충돌 방지를 위한 way point, z1 & gripper 방향 동시 이동
                                            mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return                
                code = self._arm.set_position(*[aruco_x, aruco_y, 153, 43.8, -88.5, 46.2], speed=self._tcp_speed,     # z1 & gripper 방향 동시 이동
                                            mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
            else:
                code = self._arm.set_position(*[aruco_x, aruco_y, 153, 24.8, -89.5, 154.1], speed=self._tcp_speed,    # z1 & gripper 방향 동시 이동
                                        mvacc=self._tcp_acc, radius=20.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
            time.sleep(2)
            code = self._arm.close_lite6_gripper()                      # gripper close
            if not self._check_code(code, 'close_lite6_gripper'):
                return
            time.sleep(4)            
            code = self._arm.set_position(*[170.9, 1.6, 280.1, 115.2, -88.9, 69.1], speed=self._tcp_speed,    # +x 홈으로 복귀
                                    mvacc=self._tcp_acc, radius=20.0, wait=True)
            code = self._arm.set_servo_angle(angle=[-1.6, -26.2, 7.2, 187.1, 23.4, 182.9], speed=self._angle_speed,    # 쓰레기 위치로 회전
                                                mvacc=self._angle_acc, wait=True, radius=20.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            if not self._check_code(code, 'set_position'):
                return            

            time.sleep(2)
                
# 쓰레기 버리기 & 홈으로 복귀
        code = self._arm.set_servo_angle(angle=[61.4, -26.2, 7.2, 187.1, 21.9, 182.9], speed=self._angle_speed,    # 쓰레기 위치로 회전
                                            mvacc=self._angle_acc, wait=True, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        code = self._arm.set_servo_angle(angle=[61.4, -26.2, 10.8, 187.1, 21.9, 182.9], speed=self._angle_speed,   # 쓰레기 버리기 사전 작동 angle 수정
                                            mvacc=self._angle_acc, wait=True, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)         
        code = self._arm.set_servo_angle(angle=[61.4, -47.8, 10.8, 187.1, 21.9, 182.9], speed=self._angle_speed,   # 쓰레기 버리기 사전 작동 angle 수정
                                            mvacc=self._angle_acc, wait=True, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1) 
        code = self._arm.open_lite6_gripper()           # gripper open
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.stop_lite6_gripper()           # gripper stop
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.set_servo_angle(angle=[61.4, -47.8, 7.2, 187.1, 21.9, 182.9], speed=self._angle_speed,    # 쓰레기 버린 후 angle 수정
                                            mvacc=self._angle_acc, wait=True, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        code = self._arm.set_servo_angle(angle=[61.4, -26.2, 7.2, 187.1, 21.9, 182.9], speed=self._angle_speed,    # 쓰레기 버린 후 angle 수정
                                            mvacc=self._angle_acc, wait=True, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        code = self._arm.set_servo_angle(angle=[171.6, -26.2, 7.2, 187.1, 21.9, 182.9], speed=self._angle_speed,   # 홈 위치로 회전
                                            mvacc=self._angle_acc, wait=True, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(1)
        code = self._arm.set_position(-170, 18.6, 281.9, 43.8, -88.5, -46.2, speed=50, mvacc=100, wait=True, relative=False)    # 홈 위치로 복귀
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)        
        code = self._arm.set_position(-170, 18.6, 281.9, -52.1, 89.1, 126.1, speed=200, mvacc=100, wait=True, relative=False)    # gripper 180도 원상복구(문제 발생시 angle로 수정)
        if not self._check_code(code, 'set_position'):
            return
    
    ###############################
    def making(self , top , params):
	#  motion_place_capsule
	
        code = self._arm.set_servo_angle(angle=[81.0, -10.8, 6.9, 103.6, 88.6, 9.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[10, -20.8, 7.1, 106.7, 79.9, 26.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[8.4, -42.7, 23.7, 177.4, 31.6, 3.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=40.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[8.4, -32.1, 55.1, 96.6, 29.5, 81.9], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_position(*self.position_before_capsule_place, speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*self.position_capsule_place, speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_cgpio_analog(0, 0)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(2)
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        time.sleep(0.5)
        
       	# motion_grab_cup
	
        code = self._arm.set_position(*[233.4, 10.3, 471.1, -172.2, 87.3, -84.5], speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=20.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.set_servo_angle(angle=[-2.8, -2.5, 45.3, 119.8, -79.2, -18.8], speed=self._angle_speed,
                                            mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_position(*[195.0, -96.5, 200.8, -168.0, -87.1, -110.5], speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=10.0, wait=False)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*self.position_cup_grab, speed=self._tcp_speed,
                                        mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(2)

        code = self._arm.set_position(z=120, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                      wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_servo_angle(angle=[2.9, -31.0, 33.2, 125.4, -30.4, -47.2], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        code = self._arm.set_cgpio_analog(0, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        code = self._arm.set_cgpio_analog(1, 5)
        if not self._check_code(code, 'set_cgpio_analog'):
            return
        time.sleep(0.5)

	# motion_topping
	
        if top == '1':
            code = self._arm.set_servo_angle(angle=[36.6, -36.7, 21.1, 85.6, 59.4, 44.5], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return

            if params == 'C':
                code = self._arm.set_position(*self.position_topping_C, speed=self._tcp_speed,
                                              mvacc=self._tcp_acc, radius=0.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_cgpio_digital(2, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_position(z=20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                              wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_pause_time(3)	# topping motor 시간
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_pause_time(3)	# press를 미리 구동
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(2, 0, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return

                code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc,
                                              relative=True, wait=False)
                if not self._check_code(code, 'set_position'):
                    return

            elif params == 'B':
                code = self._arm.set_servo_angle(angle=[55.8, -48.2, 14.8, 86.1, 60.2, 58.7], speed=self._angle_speed,
                                                 mvacc=self._angle_acc, wait=False, radius=20.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                code = self._arm.set_servo_angle(angle=self.position_topping_B, speed=self._angle_speed,
                                                 mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                code = self._arm.set_cgpio_digital(1, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_position(z=20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                              wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_pause_time(3)	# topping motor 시간
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_pause_time(4) # 줄이?
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(1, 0, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc,
                                              relative=True, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_servo_angle(angle=[87.5, -48.2, 13.5, 125.1, 44.5, 46.2], speed=self._angle_speed,
                                                 mvacc=self._angle_acc, wait=False, radius=10.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                code = self._arm.set_position(*[43.6, 137.9, 350.1, -92.8, 87.5, 5.3], speed=self._tcp_speed,
                                              mvacc=self._tcp_acc, radius=10.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return

            elif params == 'A':
                code = self._arm.set_position(*self.position_topping_A, speed=self._tcp_speed,
                                              mvacc=self._tcp_acc, radius=0.0, wait=True)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_cgpio_digital(0, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_pause_time(3) 
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_pause_time(1)
                if not self._check_code(code, 'set_pause_time'):
                    return
                code = self._arm.set_cgpio_digital(0, 0, delay_sec=0)
                if not self._check_code(code, 'set_cgpio_digital'):
                    return
                code = self._arm.set_servo_angle(angle=[130.0, -33.1, 12.5, 194.3, 51.0, 0.0], speed=self._angle_speed,
                                                 mvacc=self._angle_acc, wait=True, radius=0.0)
                if not self._check_code(code, 'set_servo_angle'):
                    return
                code = self._arm.set_position(*[-38.2, 132.2, 333.9, -112.9, 86.3, -6.6], speed=self._tcp_speed,
                                              mvacc=self._tcp_acc, radius=10.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
                code = self._arm.set_position(*[43.6, 137.9, 350.1, -92.8, 87.5, 5.3], speed=self._tcp_speed,
                                              mvacc=self._tcp_acc, radius=10.0, wait=False)
                if not self._check_code(code, 'set_position'):
                    return
            code = self._arm.set_position(*self.position_icecream_with_topping, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        else:
            code = self._arm.set_cgpio_digital(3, 1, delay_sec=0)
            if not self._check_code(code, 'set_cgpio_digital'):
                return
            code = self._arm.set_servo_angle(angle=self.position_icecream_no_topping, speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
        time.sleep(0.5)
        
        # motion_make_icecream
        ## 확인 필요
        if top == '1':
            time.sleep(5)	# topping -> capsule 이동 후 아이스크림이 내려오는 시간 계산
        else:
            time.sleep(8)	# topping을 받지 않는 경우 topping의 모터 시간(sleep)만큼 추가 
        time.sleep(4)
        code = self._arm.set_position(z=-20, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                      wait=True)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(4)
        code = self._arm.set_position(z=-10, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                      wait=True)
        if not self._check_code(code, 'set_position'):
            return
        if not self._check_code(code, 'set_pause_time'):
            return
        code = self._arm.set_position(z=-50, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                      wait=True)
        if not self._check_code(code, 'set_position'):
            return
        time.sleep(1)
        code = self._arm.set_cgpio_digital(3, 0, delay_sec=0)
        if not self._check_code(code, 'set_cgpio_digital'):
            return
        time.sleep(0.5)
        
        # motion_serve
        
        code = self._arm.set_servo_angle(angle=[18.2, -12.7, 8.3, 90.3, 88.1, 23.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=20.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[146.9, -12.7, 8.3, 91.0, 89.3, 22.1], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        self._tcp_speed = 100
        self._tcp_acc = 1000

        if params == 'A':
            code = self._arm.set_position(*self.position_jig_A_serve, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

            code = self._arm.set_position(z=-18, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                          wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-256.2, -126.6, 210.1, -179.2, 77.2, 66.9], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(0.5)
            code = self._arm.set_position(*[-242.8, -96.3, 210.5, -179.2, 77.2, 66.9], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-189.7, -26.0, 193.3, -28.1, 88.8, -146.0], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return

        elif params == 'B':

            code = self._arm.set_position(*self.position_jig_B_serve, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=False)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(z=-13, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                          wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-165.0, -122.7, 200, -178.7, 80.7, 92.5], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(0.5)
            code = self._arm.set_position(*[-165.9, -81.9, 200, -178.7, 80.7, 92.5], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-168.5, -33.2, 192.8, -92.9, 86.8, -179.3], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
                
        elif params == 'C':
            code = self._arm.set_servo_angle(angle=[177.6, 0.2, 13.5, 70.0, 94.9, 13.8], speed=self._angle_speed,
                                             mvacc=self._angle_acc, wait=True, radius=0.0)
            if not self._check_code(code, 'set_servo_angle'):
                return
            code = self._arm.set_position(*self.position_jig_C_serve, speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(z=-12, radius=0, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                          wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.open_lite6_gripper()
            if not self._check_code(code, 'open_lite6_gripper'):
                return
            time.sleep(1)
            code = self._arm.set_position(*[-75, -132.8, 208, -176.8, 76.1, 123.0], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.stop_lite6_gripper()
            if not self._check_code(code, 'stop_lite6_gripper'):
                return
            time.sleep(0.5)
            code = self._arm.set_position(*[-92.0, -107.5, 208, -176.8, 76.1, 123.0], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
            code = self._arm.set_position(*[-98.1, -52.1, 191.4, -68.4, 86.4, -135.0], speed=self._tcp_speed,
                                          mvacc=self._tcp_acc, radius=0.0, wait=True)
            if not self._check_code(code, 'set_position'):
                return
        time.sleep(0.5)
        code = self._arm.set_servo_angle(angle=[169.6, -8.7, 13.8, 85.8, 93.7, 19.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return

        self._tcp_speed = 100
        self._tcp_acc = 1000

	# motion_trash_capsule
	
        self._angle_speed = 150
        self._angle_acc = 300
        code = self._arm.set_servo_angle(angle=[51.2, -8.7, 13.8, 95.0, 86.0, 17.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[-16.2, -19.3, 42.7, 82.0, 89.1, 55.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        code = self._arm.set_servo_angle(angle=[-19.9, -19.1, 48.7, 87.2, 98.7, 60.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_position(*[222.8, 0.9, 470.0, -153.7, 87.3, -68.7], speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.set_position(*self.position_capsule_grab, speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        code = self._arm.close_lite6_gripper()
        if not self._check_code(code, 'close_lite6_gripper'):
            return
        time.sleep(1)
        code = self._arm.set_position(z=30, radius=-1, speed=self._tcp_speed, mvacc=self._tcp_acc, relative=True,
                                      wait=True)
        if not self._check_code(code, 'set_position'):
            return
        self._tcp_speed = 100
        self._tcp_acc = 1000
        code = self._arm.set_position(*[221.9, -5.5, 500.4, -153.7, 87.3, -68.7], speed=self._tcp_speed,
                                      mvacc=self._tcp_acc, radius=0.0, wait=True)
        if not self._check_code(code, 'set_position'):
            return
        self._angle_speed = 60
        self._angle_acc = 100
        code = self._arm.set_servo_angle(angle=[-10.7, -2.4, 53.5, 50.4, 78.1, 63.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=10.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        self._angle_speed = 160
        self._angle_acc = 1000
        code = self._arm.set_servo_angle(angle=[18.0, 11.2, 40.4, 90.4, 58.7, -148.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.open_lite6_gripper()
        if not self._check_code(code, 'open_lite6_gripper'):
            return
        code = self._arm.set_servo_angle(angle=[25.2, 15.2, 42.7, 83.2, 35.0, -139.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[18.0, 11.2, 40.4, 90.4, 58.7, -148.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[25.2, 15.2, 42.7, 83.2, 35.0, -139.8], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.stop_lite6_gripper()
        if not self._check_code(code, 'stop_lite6_gripper'):
            return
        self._angle_speed = 120
        self._angle_acc = 1000
        code = self._arm.set_servo_angle(angle=[28.3, -9.0, 12.6, 85.9, 78.5, 20.0], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=30.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[149.3, -9.4, 10.9, 114.7, 69.1, 26.1], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=False, radius=50.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        code = self._arm.set_servo_angle(angle=[179.2, -42.1, 7.4, 186.7, 41.5, -1.6], speed=self._angle_speed,
                                         mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(0.5)

    def trash_detect(self):		# home위치로 먼저 이동 후 쓰레기 detect를 위해 이동
        code = self._arm.set_servo_angle(angle=[171.3, -26.3, 7.3, 171.7, 57.2, 3.9], speed=self._angle_speed,
                                        mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        time.sleep(0.5)
        code = self._arm.set_servo_angle(angle=[270, -26.3, 7.3, 171.7, 57.2, 3.9], speed=self._angle_speed,
                                        mvacc=self._angle_acc, wait=True, radius=0.0)
        if not self._check_code(code, 'set_servo_angle'):
            return
        # time.sleep(3)
        # code = self._arm.set_servo_angle(angle=[171.3, -26.3, 7.3, 171.7, 57.2, 3.9], speed=self._angle_speed,
        #                                 mvacc=self._angle_acc, wait=True, radius=0.0)
        # if not self._check_code(code, 'set_servo_angle'):
        #     return
        # time.sleep(0.5)


class RobotControl(Node):
    def __init__(self , robot_main):
        super().__init__('RobotControl')

        # Define callback groups
        self.topic_callback_group = ReentrantCallbackGroup()
        self.service_callback_group = ReentrantCallbackGroup()

        # Robot_Server에서 토픽 받기 (호객행위 / 환영인사 / 작별인사 / 중지)
        self.robot_server_subscriber = self.create_subscription(
            TopicString,
            '/Server_to_Robot',
            self.robot_server_callback,
            10,
            callback_group=self.topic_callback_group
        )

        # prevent unused variable warning
        self.robot_server_subscriber

        # Robot_Server에서 /Call_to_Robot 서비스를 받아옴 (쓰레기 처리 - 좌표 / 통 잡기 , 실링 확인 / 자리복귀 / 아이스크림 제조)
        self.robot_control_server = self.create_service(
            ServiceString,
            '/Call_to_Robot',
            self.robot_control_callback,
            callback_group=self.service_callback_group
            )
        
        self.Robot_Control = robot_main
        

    def robot_server_callback(self, msg):
        self.get_logger().info(f'Received command: {msg.command}')
        if msg.command == "human_detect":
            self.get_logger().info("호객행위") # come_on
            self.Robot_Control.motion_come_on()
        elif msg.command == "guest_detect":
            self.get_logger().info("환영인사") # greet
            self.Robot_Control.motion_greet()
            print("ok1")
            self.Robot_Control.motion_home()
            print("ok2") 
        elif msg.command == "bye":
            self.get_logger().info("작별인사") # greet
            self.Robot_Control.motion_greet()
            self.Robot_Control.motion_home() 
        elif msg.command == "home":
            self.get_logger().info("홈복귀") # greet
            self.Robot_Control.motion_home()
        elif msg.command == "init":
            self.get_logger().info("강제종료") # 
            self.Robot_Control._arm.set_state(4)
        elif msg.command == "stop":
            self.get_logger().info("중지") # set_state (3)
            self.Robot_Control._arm.set_state(3)
        elif msg.command == "restart":
            self.get_logger().info("재시작") # set_state (0)
            self.Robot_Control._arm.set_state(0)
        elif "trash_detected" in msg.command:
            self.get_logger().info("쓰레기처리")  # trash + 좌표(x,y)
            parts = msg.command.split(',')
            command = parts[0].strip()
            x = float(parts[1].strip())  # 문자열을 float으로 변환
            y = float(parts[2].strip())  # 문자열을 float으로 변환
            print(x, y)
            threading.Thread(target=self.Robot_Control.trash, args=(x, y)).start()

        else:
            self.get_logger().info("Uncommand!") 
            return
    
    def robot_control_callback(self, request, response):
        self.get_logger().info(f'Received command: {request.command}')

        # 쓰레기 확인
        if request.command == "trash_pre":
            self.get_logger().info("쓰레기위치")
            self.Robot_Control.trash_detect()

        elif "trash_detected" in request.command:
            self.get_logger().info("쓰레기처리")  # trash + 좌표(x,y)
            parts = request.command.split(',')
            command = parts[0].strip()
            x = float(parts[1].strip())  
            y = float(parts[2].strip()) 
            print(x, y)
            self.Robot_Control.motion_home()
            threading.Thread(target=self.Robot_Control.trash, args=(x, y)).start()

        elif "pre_making" in request.command:
            parts = request.command.split(',')
            command = parts[0].strip()
            param = parts[1].strip()
            print(param)

            self.get_logger().info("통잡기") # motion_grab_capsule + A,B,C 전달
            self.Robot_Control.motion_home()
            self.Robot_Control.motion_grab_capsule(param)

            self.get_logger().info("실링확인") # sealing_check + A,B,C 전달
            self.Robot_Control.sealing_check(param)

        elif "return" in request.command:
            self.get_logger().info("실링 실패") # motion_place_fail_capsule
            parts = request.command.split(',')
            command = parts[0].strip()
            param = parts[1].strip()
            print(param)
            self.Robot_Control.motion_place_fail_capsule(param)
            self.Robot_Control.motion_home()

        elif "making" in request.command:
            parts = request.command.split(',')
            command = parts[0].strip()
            top = parts[1].strip()
            param = parts[2].strip()
            print(top , param)

            self.Robot_Control.making(top,param)
        else:
            self.get_logger().error("ERROR")

        response.success = True
        response.result = f'Command {request.command} received and being processed' 

        print(response)

        return response

def main(args=None):
    RobotMain.pprint('xArm-Python-SDK Version:{}'.format(version.__version__))
    arm = XArmAPI('192.168.1.192', baud_checkset=False)
    robot_main = RobotMain(arm)

    socket_thread = Thread(target=robot_main.socket_connect)
    socket_thread.start()
    print('socket_thread start')

    joint_state_thread = threading.Thread(target=robot_main.joint_state)
    joint_state_thread.start()
    print('joint_state_thread_started')

    rp.init(args=args)

    Robot_Control = RobotControl(robot_main)

    # MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(Robot_Control)

    try:
        executor.spin()
    finally:
        Robot_Control.destroy_node()
        rp.shutdown()

if __name__ == '__main__':
    main()