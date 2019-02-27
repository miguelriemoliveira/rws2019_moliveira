#!/usr/bin/env python
"""
This is a long, multiline description
"""

# ------------------------
#   IMPORTS
# ------------------------
import os
import signal

import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import String
from rws2019_msgs.msg import MakeAPlay
# from rws2018_msgs.srv import GameQuery
import random
import tf
import math
import subprocess
from sensor_msgs.msg import Image, PointCloud2, PointField

import rospkg


# ------------------------
#   DATA STRUCTURES
# ------------------------
class Player:

    def __init__(self, name):
        self.name = name
        self.team = self.checkTeam()
        self.x = random.random() * 1000 + 5000
        self.y = random.random() * 1000 + 5000
        self.rot = (0, 0, 0, 1)
        self.stamp_last_pose = None
        self.stamp_resuscitated = rospy.Time.now()
        self.stamp_killed = rospy.Time.now() - rospy.Duration.from_sec(10)
        self.num_hunted = 0
        self.num_preyed = 0
        self.score = 0
        self.processes = [] # list of processes for this player

    def checkTeam(self):
        """
        Checks to which team a player belongs to
        :return: a String containing the name of the team
        """
        if self.name in rospy.get_param('/team_red'):
            return 'red'
        elif self.name in rospy.get_param('/team_green'):
            return 'green'
        elif self.name in rospy.get_param('/team_blue'):
            return 'blue'

    def resuscitate(self):
        for process in self.processes:
            process.kill()
        self.processes = []

        cmd = "rosrun player_" + self.name + ' player_' + self.name + '_node'
        # p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # p = subprocess.Popen(cmd, stdout=subprocess.PIPE,  stderr=subprocess.PIPE,
        #                        shell=True, preexec_fn=os.setsid)

        p = subprocess.Popen("exec " + cmd, stdout=subprocess.PIPE, shell=True)
        self.processes.append(p)
        self.stamp_resuscitated = rospy.Time.now()

    def kill(self):
        cmd = "rosnode kill " + player
        # p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # rospy.loginfo("Waiting to rosnode kill " + self.name)
        # p.wait()
        # p.kill()

        for process in self.processes:
            process.kill()
            # os.killpg(os.getpgid(process.pid), signal.SIGTERM)

        self.processes = []

        self.stamp_killed = rospy.Time.now()

    def updatePose(self):
        try:
            (trans, rot) = listener.lookupTransform("/world", self.name, rospy.Time(0))
            self.x = trans[0]
            self.y = trans[1]
            self.rot = rot
            self.stamp_last_pose = rospy.get_time()

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            # rospy.logwarn("Could not get pose of player " + str(player))
            pass

    def __str__(self):
        s = String()
        a = '[x,y] = [' + str(self.x) + ', ' + str(self.y) + '] at time ' + str(self.stamp_last_pose) + '\n'
        a += 'Team ' + self.team + ' killed at ' + str(self.stamp_killed) + ' resuscitated at ' + str(
            self.stamp_resuscitated) + '\n'
        a += 'hunted ' + str(self.num_hunted) + ' players and was hunted ' + str(self.num_preyed) + ' times'
        return a


# ------------------------
# GLOBAL VARIABLES
# ------------------------
score = {'red': 0, 'green': 0, 'blue': 0}
pub_make_a_play = rospy.Publisher('make_a_play', MakeAPlay, queue_size=10)
pub_referee = rospy.Publisher("referee_markers", MarkerArray, queue_size=10)
pub_score = rospy.Publisher("score_markers", MarkerArray, queue_size=10)
pub_rip = rospy.Publisher("kill_markers", MarkerArray, queue_size=10)
pub_killer = rospy.Publisher("victim", String, queue_size=10)
pub_players = rospy.Publisher("player_info", MarkerArray, queue_size=10)
rospy.init_node('referee', anonymous=False)  # initialize ros node
listener = tf.TransformListener()
broadcaster = tf.TransformBroadcaster()

rate = 0
game_duration = rospy.get_param('/game_duration')
positive_score = rospy.get_param('/positive_score')
negative_score = rospy.get_param('/negative_score')
killed = []
team_red = []
team_green = []
team_blue = []
player_score_pos = dict()
player_score_neg = dict()
selected_team_count = 0
game_pause = False
game_over = False
to_print_end = True
pinfo = {}  # stores information about the position of players
cheetah_speed = 0.1
turtle_speed = 0.1
cat_speed = 0.1
dog_speed = 0.1


# ------------------------
# FUNCTION DEFINITION
# ------------------------
##
# @brief Executes the command in the shell in a blocking or non-blocking manner
#
# @param cmd a string with teh command to execute
#
# @return
def bash(cmd, blocking=True):
    # print "Executing command: " + cmd
    # p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

    # if blocking or True:
    if blocking:
        for line in p.stdout.readlines():
            print line,
            p.wait()


def createMarker(frame_id, type, action=Marker.ADD, ns='', id=0, lifetime=rospy.Duration(0), frame_locked=0,
                 stamp=rospy.Time.now(),
                 position_x=0, position_y=0, position_z=0,
                 orientation_x=0, orientation_y=0, orientation_z=0, orientation_w=0,
                 scale_x=1, scale_y=1, scale_z=1,
                 color_r=0, color_g=0, color_b=0, color_a=1,
                 text=''):
    m = Marker()

    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.type = type
    m.action = action
    m.id = id
    m.ns = ns
    m.lifetime = lifetime
    m.frame_locked = frame_locked

    m.pose.position.x = position_x
    m.pose.position.y = position_y
    m.pose.position.z = position_z

    m.pose.orientation.x = orientation_x
    m.pose.orientation.y = orientation_y
    m.pose.orientation.z = orientation_z
    m.pose.orientation.w = orientation_w

    m.scale.x = scale_x
    m.scale.y = scale_y
    m.scale.z = scale_z

    m.color.r = color_r
    m.color.g = color_g
    m.color.b = color_b
    m.color.a = color_a

    m.text = text
    m.frame_locked = frame_locked

    return m


# ------------------------
# BASE CLASSES
# ------------------------

# ------------------------
# CLASS DEFINITION
# ------------------------

def gameQueryCallback(event):
    global team_red, team_green, team_blue, selected_team_count, game_pause, score
    game_pause = True

    rospy.loginfo("gameQueryCallback")
    rospy.loginfo("selected_team_count = " + str(selected_team_count))
    # return None

    # percorrer a lista de equipas
    team_list = [team_red, team_green, team_blue]

    selected_team = team_list[selected_team_count]

    # print("team_list is = " + str(team_list))
    # print("selected_team is = " + str(selected_team))

    # sortear um jogador alive da equipa desta iteracao

    selected_player = random.choice(selected_team)
    # selected_player = "moliveira"
    print("selected_player is = " + str(selected_player))

    # sortear um objeto
    objects = ["banana", "soda_can", "onion", "tomato"]
    selected_object = random.choice(objects)

    rospack = rospkg.RosPack()
    path_pcd = rospack.get_path('rws2018_referee') + "/../pcd/"
    file_pcd = path_pcd + selected_object + ".pcd"
    # print("vou ler o " + str(file_pcd))

    # pedir ao pcd2pointcloud para enviar o objeto

    cmd = "rosrun rws2018_referee pcd2pointcloud _input:=" + file_pcd + " _output:=/object_point_cloud /world:=" + selected_player + " _one_shot:=1"
    # print "Executing command: " + cmd
    p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    for line in p.stdout.readlines():
        print line,
        p.wait()

    # sleep for duration (to make sure people get the point clouds)
    d = rospy.Duration(2, 0)
    rospy.sleep(d)

    # chamar o servico game_query
    service_name = "/" + selected_player + "/game_query"
    correct_response = False

    try:
        rospy.wait_for_service(service_name, 1)
    except rospy.ROSException, e:
        print("Perguntei " + selected_object + " ao " + selected_player + " e ele(a) nao deu resposta")
        print("RESPOSTA AUSENTE!  ...")

    try:
        game_query = rospy.ServiceProxy(service_name, GameQuery)
        resp1 = game_query()
        print("Perguntei " + selected_object + " ao " + selected_player + " e ele respondeu " + resp1.resposta)
        # verificar a resposta e afetar a pontuacao
        if selected_object == resp1.resposta:
            print("RESPOSTA CERTA! FANTASTICO")
            correct_response = True
        else:
            print("RESPOSTA ERRADA! NAO PERCEBES NADA DISTO ...")
    except rospy.ServiceException, e:
        print "Service call failed: %s" % e

    print("score before:" + str(score))
    if correct_response == True:
        score[selected_team_count] = score[selected_team_count] + 5
    else:
        score[selected_team_count] = score[selected_team_count] - 5

    print("score after:" + str(score))

    if selected_team_count == 2:
        selected_team_count = 0
    else:
        selected_team_count = selected_team_count + 1

    # sleep for duration (to make sure people get the point clouds)
    rospy.sleep(d)
    game_pause = False


def randomizeVelocityProfiles(event):
    global cheetah_speed, turtle_speed, dog_speed, cat_speed
    cheetah_speed = random.random() / 10
    turtle_speed = random.random() / 10
    dog_speed = random.random() / 10
    cat_speed = random.random() / 10


def makeAPlayCallback(event):
    """ Publishes a makeAPlay message

    :param event:
    :return:
    """
    global game_pause, game_over, team_red, team_green, team_blue, killed

    if game_pause == True or game_over:  # do not publish if game is over or paused
        rospy.logwarn("Game is paused or over!!!")
        if game_over:
            printScores()
        return

    # Define the message
    a = MakeAPlay()  # Create a MakeAPlay message

    print("killed: " + str(killed))

    # Find on which team the killed are
    for player in team_red:
        if player in killed:
            a.red_dead.append(player)
        else:
            a.red_alive.append(player)

    for player in team_green:
        if player in killed:
            a.green_dead.append(player)
        else:
            a.green_alive.append(player)

    for player in team_blue:
        if player in killed:
            a.blue_dead.append(player)
        else:
            a.blue_alive.append(player)

    # Randomize velocity profiles
    a.cheetah = cheetah_speed
    a.dog = dog_speed
    a.cat = cat_speed
    a.turtle = turtle_speed

    # Publish MakeAPlay msg
    if not rospy.is_shutdown():
        rospy.loginfo("Publishing a make a play")
        pub_make_a_play.publish(a)


def printScores():
    global to_print_end
    if to_print_end:

        # for player in player_score_pos:
        for player, score in player_score_neg.iteritems():
            score_neg = player_score_neg[player]
            score_pos = player_score_pos[player]
            score_diff = score_pos - score_neg
            rospy.loginfo(player + ": killed: " + str(score_pos) + " died: " + str(score) + " diff: " + str(score_diff))
        # for player,score in player_score_neg.iteritems():
        # rospy.loginfo(player + " died: " + str(score))
        # for player,score in player_score_pos.iteritems():
        # rospy.loginfo(player + " killed: " +  str(score))
        # for player,score in player_score_dif.iteritems():
        # rospy.loginfo(player + " dif: " +  str(score))
        to_print_end = False


def gameEndCallback(event):
    """
    Called after the game time expires. Show print winning team along with additional info.
    :param event: timer event
    """
    rospy.logwarn("\n\n\nGame finished\n\n\n")
    global pub_score, game_over, score
    game_over = True

    ma = MarkerArray()

    if score['red'] > score['green'] and score['red'] > score['blue']:
        text = "Team R wins the game"
    if score['green'] > score['red'] and score['green'] > score['blue']:
        text = "Team G wins the game"
    if score['blue'] > score['red'] and score['blue'] > score['green']:
        text = "Team B wins the game"
    else:
        text = "WHAT HAPPENNED?"

    m = createMarker(frame_id='/world', type=Marker.TEXT_VIEW_FACING, id=777, scale_x=.2, scale_y=.2, scale_z=.9,
                     color_r=.1, color_g=.1, color_b=.1, position_x=.5, position_y=4.5, text=text)

    ma.markers.append(m)
    pub_score.publish(ma)



def printPInfo():
    global pinfo
    for key in pinfo:
        print('Player ' + key + ' has properties:')
        print(pinfo[key])


def checkGame(event):
    global listener, broadcaster, pinfo, immunity_duration, hunting_distance, max_distance_from_center_of_arena
    global game_duration
    global team_red, team_green, team_red
    global pub_score, game_over

    if game_over:  # if game over no need to check
        return

    # -----------------------------
    # Initialize lists
    # -----------------------------
    max_distance_from_center_of_arena = 8
    to_be_killed = []
    rospy.loginfo("killed players " + str(killed))
    ma_killed = MarkerArray()
    ma_arena = MarkerArray()
    ma_players = MarkerArray()

    # -----------------------------
    # Resuscitating players
    # -----------------------------
    tic = rospy.Time.now()
    for player in killed:
        if (rospy.Time.now() - pinfo[player].stamp_killed).to_sec() > killed_duration:
            killed.remove(player)
            pinfo[player].resuscitate()
            rospy.logwarn("Ressuscitating %s", player)
            # broadcaster.sendTransform((random.random() * 10 - 5, random.random() * 10 - 5, 0),
            #                           tf.transformations.quaternion_from_euler(0, 0, 0), tic, player[0], "/world")

    # -----------------------------
    # Get the pose of all players
    # -----------------------------
    for player in team_red + team_green + team_blue:
        pinfo[player].updatePose()

    # printPInfo()

    # -----------------------------
    # Check if anyone is hunted
    # -----------------------------
    z = zip(['red', 'green', 'blue'], ['green', 'blue', 'red'], [team_red, team_green, team_blue],
            [team_green, team_blue, team_red])
    for hunter_color, prey_color, hunters, preys in z:

        # print("Checking for hunter team " + hunter_color + " and prey team " + prey_color)
        for hunter in hunters:
            # print("hunter " + hunter)
            if hunter in killed:  # if hunter is killed he cannot hunt
                continue
            for prey in preys:
                if prey in killed or prey in to_be_killed:  # cannot hunt already killed or to be killed prey
                    continue

                # Cannot hunt during the immunity time
                if (rospy.Time.now() - pinfo[prey].stamp_resuscitated).to_sec() < immunity_duration:
                    continue

                distance = math.sqrt((pinfo[hunter].x - pinfo[prey].x) ** 2 + (pinfo[hunter].y - pinfo[prey].y) ** 2)

                if distance < hunting_distance:
                    to_be_killed.append(prey)

                    # update hunter score
                    pinfo[hunter].num_hunted += 1
                    pinfo[hunter].score += positive_score

                    # update prey score
                    pinfo[prey].num_preyed += 1
                    pinfo[prey].score += negative_score

                    # update team scores
                    score[hunter_color] = score[hunter_color] + positive_score
                    score[prey_color] = score[prey_color] + negative_score

                    # print and draw
                    rospy.logwarn(prey + " (" + prey_color + ") was hunted by " + hunter + "(" + hunter_color + ")")
                    ma_killed.markers.append(
                        createMarker(frame_id="/world", type=Marker.TEXT_VIEW_FACING, ns=prey,
                                     lifetime=rospy.Duration.from_sec(5),
                                     position_x=pinfo[prey].x, position_y=pinfo[prey].y,
                                     scale_z=0.4, color_r=0.8, color_g=0.6, color_b=0.2, color_a=1,
                                     text='RIP ' + prey + ' (by ' + hunter + ')'))

    # --------------------------------
    # Killing because of stray from arena
    # --------------------------------
    for player in team_red + team_green + team_blue:
        if player in killed:  # if player is killed he cannot be killed for straying the arena
            continue

        # Cannot stray from arena during immunity time
        if (rospy.Time.now() - pinfo[player].stamp_resuscitated).to_sec() < immunity_duration:
            continue

        distance = math.sqrt(pinfo[player].x ** 2 + pinfo[player].y ** 2)

        if distance > max_distance_from_center_of_arena:
            to_be_killed.append(player)

            # update player score
            pinfo[player].num_preyed += 1
            pinfo[player].score += negative_score

            # update team score
            color = pinfo[player].team
            score[color] = score[color] + negative_score

            # print and draw
            rospy.logwarn(player + " (" + color + ") strayed away from the arena.")
            ma_killed.markers.append(
                createMarker(frame_id="/world", type=Marker.TEXT_VIEW_FACING, ns=player,
                             lifetime=rospy.Duration.from_sec(5),
                             position_x=pinfo[player].x, position_y=pinfo[player].y,
                             scale_z=0.4, color_r=0.2, color_g=0.2, color_b=0.2, color_a=1,
                             text='You are a chiken ' + player))

    # --------------------------------
    # Kill players
    # --------------------------------
    # rospy.loginfo("to_be_killed = " + str(to_be_killed))
    kill_time = rospy.Time.now()
    for player in to_be_killed:
        if player in killed:  # cannot kill an already killed player
            to_be_killed.remove(player)
            continue

        rospy.logwarn("Killing " + str(player))

        pinfo[player].kill()

        to_be_killed.remove(player)  # remove from list to be killed
        killed.append(player)

    # --------------------------------
    # Update transformation for killed players
    # --------------------------------
    for player in killed:
        broadcaster.sendTransform((random.random() * 1000 + 5000, random.random() * 1000 + 5000, 0),
                                  tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(),
                                  player, "/world")

    # --------------------------------
    # Create arena markers
    # --------------------------------

    ma_arena.markers.append(
        createMarker(frame_id="/world", type=Marker.TEXT_VIEW_FACING, id=0, position_x=-5, position_y=5.2, scale_z=.6,
                     color_r=1, color_a=1, text="R=" + str(score['red']), lifetime=rospy.Duration.from_sec(0)))

    ma_arena.markers.append(
        createMarker(frame_id="/world", type=Marker.TEXT_VIEW_FACING, id=1, position_x=-3, position_y=5.2, scale_z=.6,
                     color_g=1, color_a=1, text="G=" + str(score['green']), lifetime=rospy.Duration.from_sec(0)))

    ma_arena.markers.append(
        createMarker(frame_id="/world", type=Marker.TEXT_VIEW_FACING, id=2, position_x=-1, position_y=5.2, scale_z=.6,
                     color_b=1, color_a=1, text="B=" + str(score['blue']), lifetime=rospy.Duration.from_sec(0)))

    ma_arena.markers.append(
        createMarker(frame_id="/world", type=Marker.TEXT_VIEW_FACING, id=3, position_x=3.5, position_y=5.2, scale_z=.6,
                     color_b=1, color_a=1,
                     text="Time " + str(format((rospy.Time.now() - game_start).to_sec(), '.2f')) + " of " + str(
                         game_duration)))

    # --------------------------------
    # Create player markers
    # --------------------------------
    for player in team_red + team_green + team_blue:
        color = pinfo[player].team
        if color == 'red':
            color_r, color_g, color_b = 1, 0, 0
        elif color == 'green':
            color_r, color_g, color_b = 0, 1, 0
        else:
            color_r, color_g, color_b = 0, 0, 1

        # Draw text with player name
        ma_players.markers.append(
            createMarker(frame_id='/world', type=Marker.TEXT_VIEW_FACING, id=0, ns=player, scale_z=.4,
                         position_x=pinfo[player].x, position_y=pinfo[player].y,
                         color_r=color_r, color_g=color_g, color_b=color_b, color_a=1,
                         text=player))

        # Draw an arrow
        ma_players.markers.append(
            createMarker(frame_id='/world', type=Marker.ARROW, id=1, ns=player,
                         scale_x=.5, scale_y=.1, scale_z=.1,
                         position_x=pinfo[player].x, position_y=pinfo[player].y,
                         orientation_x=pinfo[player].rot[0], orientation_y=pinfo[player].rot[1],
                         orientation_z=pinfo[player].rot[2], orientation_w=pinfo[player].rot[3],
                         color_r=color_r, color_g=color_g, color_b=color_b, color_a=.7))

        # Draw a circle of hunting distance
        ma_players.markers.append(
            createMarker(frame_id='/world', type=Marker.CYLINDER, id=2, ns=player,
                         scale_x=2 * hunting_distance, scale_y=2 * hunting_distance, scale_z=.01,
                         position_x=pinfo[player].x, position_y=pinfo[player].y, position_z=-.2,
                         orientation_x=pinfo[player].rot[0], orientation_y=pinfo[player].rot[1],
                         orientation_z=pinfo[player].rot[2], orientation_w=pinfo[player].rot[3],
                         color_r=color_r, color_g=color_g, color_b=color_b, color_a=.2))

        # Draw a bright circle signalizing immunity
        if (rospy.Time.now() - pinfo[player].stamp_resuscitated).to_sec() < immunity_duration:
            ma_players.markers.append(
                createMarker(frame_id='/world', type=Marker.CYLINDER, id=3, ns=player,
                             scale_x=3 * hunting_distance, scale_y=3 * hunting_distance, scale_z=.01,
                             position_x=pinfo[player].x, position_y=pinfo[player].y, position_z=-.1,
                             orientation_x=pinfo[player].rot[0], orientation_y=pinfo[player].rot[1],
                             orientation_z=pinfo[player].rot[2], orientation_w=pinfo[player].rot[3],
                             color_r=.7, color_g=.7, color_b=.7, color_a=.2, text=player))
        else:  # remove the immunity ring
            ma_players.markers.append(
                createMarker(frame_id='/world', type=Marker.CYLINDER, id=3, ns=player, action=Marker.DELETE))

    # --------------------------------
    # Publishing markers
    # --------------------------------
    if ma_killed.markers:
        pub_rip.publish(ma_killed)

    if ma_arena.markers:
        pub_score.publish(ma_arena)

    if ma_players.markers:
        pub_players.publish(ma_players)

def reAdvertise(event):
    global pub_make_a_play
    rospy.logwarn("Readvertising")
    pub_make_a_play = rospy.Publisher('make_a_play', MakeAPlay, queue_size=10)

if __name__ == '__main__':

    hunting_distance = rospy.get_param('/hunting_distance')
    immunity_duration = rospy.get_param('/immunity_duration')
    killed_duration = rospy.get_param('/killed_duration')
    rospy.sleep(0.5) # make sure the rospy time works

    # -------------------------------------
    # Create players
    # -------------------------------------
    team_red = rospy.get_param('/team_red')
    team_green = rospy.get_param('/team_green')
    team_blue = rospy.get_param('/team_blue')
    for player in team_red + team_green + team_blue:
        pinfo[player] = Player(player)

    # -------------------------------------
    # Resuscitate
    # -------------------------------------
    for player in team_red + team_green + team_blue:
        pinfo[player].resuscitate()

    # -------------------------------------
    # Update player poses
    # -------------------------------------
    rospy.sleep(0.5)
    for player in team_red + team_green + team_blue:
        pinfo[player].updatePose()

    # -------------------------------------
    # Setup periodical callbacks
    # -------------------------------------
    rospy.Timer(rospy.Duration(0.1), makeAPlayCallback, oneshot=False)
    rospy.Timer(rospy.Duration(0.5), randomizeVelocityProfiles, oneshot=False)
    rospy.Timer(rospy.Duration(0.03), checkGame, oneshot=False)
    rospy.Timer(rospy.Duration(game_duration), gameEndCallback, oneshot=True)
    # rospy.Timer(rospy.Duration(1), reAdvertise, oneshot=False)

    # rospy.Timer(rospy.Duration(25), gameQueryCallback, oneshot=False)


    game_start = rospy.Time.now()

    rospy.loginfo("Starting a game of %d secs", game_duration)

    rospy.spin()
    rospy.signal_shutdown("Game finished")
