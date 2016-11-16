# -*- coding: utf8 -*-


from bitbots_common.debug.debug cimport Scope

cdef Scope debug = Scope("Motion.Server.Standuphandler")

#from bitbots_motion_old.basemotionserver import STATE_GETTING_UP, STATE_FALLING,STATE_STARTUP,STATE_FALLEN,STATE_CONTROLABLE, STATE_SOFT_OFF, STATE_PENALTY_ANIMANTION
# TODO c&p ist doof
cdef public enum MOTION_STATES:
    STATE_CONTROLABLE = 1
    STATE_FALLING
    STATE_FALLEN
    STATE_GETTING_UP
    STATE_ANIMATION_RUNNING
    STATE_BUSY
    STATE_STARTUP
    STATE_PENALTY
    STATE_PENALTY_ANIMANTION
    STATE_RECORD
    STATE_SOFT_OFF
    STATE_WALKING

cdef class StandupHandler(object):

    def __init__(self):
        self.not_much_smoothed_gyro = DataVector(0, 0, 0)
        self.raw_gyro = IntDataVector(0,0,0)

    cdef update_sensor_data(self, raw_gyro):
        self.raw_gyro = raw_gyro
        self.not_much_smoothed_gyro = self.not_much_smoothed_gyro * 0.5 + raw_gyro * 0.5

    cdef load_falling_data(self, config) :
        self.config = config
        self.falling_activated           = config["falling"][config["RobotTypeName"]]["dynamisches_fallen_aktiviert"]
        self.falling_ground_coefficient  = config["falling"][config["RobotTypeName"]]["boden_koeffizient"]

        #Fallanimation laden
        self.falling_motor_degrees_front = config["falling"][config["RobotTypeName"]]["motor_stellungen_vorne"]
        self.falling_motor_degrees_back  = config["falling"][config["RobotTypeName"]]["motor_stellungen_hinten"]
        self.falling_motor_degrees_right = config["falling"][config["RobotTypeName"]]["motor_stellungen_rechts"]
        self.falling_motor_degrees_left  = config["falling"][config["RobotTypeName"]]["motor_stellungen_links"]

        #Fallerkennungs Grenzwerte laden
        self.falling_threshold_front     = config["falling"][config["RobotTypeName"]]["grenzwert_gyro_y_vorne"] \
                                           * self.falling_ground_coefficient + config["ZMPConfig"][config["RobotTypeName"]]["HipPitch"]
        self.falling_threshold_back      = config["falling"][config["RobotTypeName"]]["grenzwert_gyro_y_hinten"]\
                                           * self.falling_ground_coefficient + config["ZMPConfig"][config["RobotTypeName"]]["HipPitch"]
        self.falling_threshold_right     = config["falling"][config["RobotTypeName"]]["grenzwert_gyro_x_rechts"]\
                                           * self.falling_ground_coefficient
        self.falling_threshold_left      = config["falling"][config["RobotTypeName"]]["grenzwert_gyro_x_links"]\
                                           * self.falling_ground_coefficient

        #Grenzwerte an Untergrund anpassen
        self.falling_threshold_front     *= self.falling_ground_coefficient
        self.falling_threshold_back      *= self.falling_ground_coefficient
        self.falling_threshold_right     *= self.falling_ground_coefficient
        self.falling_threshold_left      *= self.falling_ground_coefficient

        return

    cdef set_falling_pose(self, object falling_motor_degrees, object goal_pose):
        if self.falling_activated:
            for i in range(1,20):
                a = falling_motor_degrees[i-1]
                goal_pose.get_joint_by_cid(i).goal = a
            pass
        else:
            goal_pose.set_active(False)


    cdef check_fallen(self, object goal_pose, object set_state,  state, smooth_gyro, robo_angle, smooth_accel):
        """ Prüfen, ob der Roboter hingefallen ist oder gerade fällt """
        # beim initialen aufstehen, und beim soft-off nicht prüfen ob er liegt
        # wenn wir gerade eine Animation Spielen (Penalty oder getting up)
        # wollen wir auch nicht aufs fallen auchten

        if state not in (
                STATE_GETTING_UP,
                STATE_SOFT_OFF,
                STATE_PENALTY_ANIMANTION):

            #Fallpruefung nach vorne bzw. nach hinten:
            #erstellt von 3doerfle, 3bimberg, 3windsor (Robocup Praktikum 2015)
            #Diskrete Werte stehen in "falling.yaml"
            # Animationen nicht ins Animationsregister übertragen um eventuelles Fehlverhalten (z.B. durch gegenseitiges
            # blockieren verschiedener States) zu vermeiden. und reaktion muss sehr schnell erfolgen.
            #GETESTET FUER: wheatly, Tamara, (Fiona)--> konnte beim test nicht richtig laufen, Goal (nur mit falling_activated = false)
            #Gerade nicht am fallen, (Animation schon aktiv)
            if state != STATE_FALLING :
                #falle ich eher zur seite oder nach vorne/hinten?
                if abs(self.not_much_smoothed_gyro.get_y()) > abs(self.not_much_smoothed_gyro.get_x()) :
                    fallingPose = self.check_fallen_forwardAndBackward(set_state, state)
                    if fallingPose:
                        set_state(STATE_FALLING)
                        state = STATE_FALLING
                        self.set_falling_pose(fallingPose, goal_pose)
                else:
                    fallingPose = self.check_fallen_sideways(set_state, state)
                    if fallingPose:
                        set_state(STATE_FALLING)
                        state = STATE_FALLING
                        self.set_falling_pose(fallingPose, goal_pose)

            #in den nächsten Fallunterscheidungen haben wir den raw_gyro mit reingenommen,
            #da sonst bei aus dem stand umfallen zu frueh auf "ich liege schon" umgeschaltet wurde
            #3doerfle, 3bimberg, 3windsor(Robocup Praktikum 2015)
            if state == STATE_FALLING and smooth_gyro.norm() < 3 and self.raw_gyro.norm() < 3: ###gyro
                if smooth_accel.norm() > 30:
                    self.info("Ich bin hingefallen")
                    set_state(STATE_FALLEN)
                    state = STATE_FALLEN
                else:
                    self.info(
                        "Ich glaube, dass ich doch nicht hingefallen bin")
                    set_state(STATE_CONTROLABLE)
                    state = STATE_CONTROLABLE
                return


            # Nicht so gut, aber funktioniert erstmal
            if self.raw_gyro.norm() < 5 and smooth_gyro.norm() < 5 and robo_angle.y > 80: ###gyro
                self.info("Ich liege auf dem Bauch, ich sollte aufstehen!")
                set_state(STATE_GETTING_UP)
                state = STATE_GETTING_UP
                self.fallState = FALLEN_FRONT_UP
                return self.config["animations"]["motion"]["front-up"]

            if self.raw_gyro.norm() < 5 and smooth_gyro.norm() < 5 and robo_angle.y < -60: ###gyro
                self.info("Ich liege auf dem Rücken, ich sollte aufstehen!")
                set_state(STATE_GETTING_UP)
                state = STATE_GETTING_UP
                self.fallState = FALLEN_BOTTOM_UP
                return self.config["animations"]["motion"]["bottom-up"]

            if self.raw_gyro.norm() < 5 and smooth_gyro.norm() < 5 and abs(robo_angle.x) > 60:
                self.info("Ich liege auf der Seite und sollte aufstehen")
                set_state(STATE_GETTING_UP)
                state = STATE_GETTING_UP
                self.fallState = FALLEN_FRONT_UP
                return self.config["animations"]["motion"]["front-up"]

            if state == STATE_FALLEN and self.raw_gyro.norm() < 3 and smooth_gyro.norm() < 3:
                set_state(STATE_GETTING_UP)
                state = STATE_GETTING_UP
                self.fallState = FALLEN_UPRIGHT
                return self.config["animations"]["motion"]["walkready"]

        if state == STATE_STARTUP:
            set_state(STATE_GETTING_UP)
            state = STATE_STARTUP
            self.fallState = FALLEN_UPRIGHT
            return self.config["animations"]["motion"]["walkready"]

    cdef check_fallen_forwardAndBackward(self, set_state , state):
        # Prüfen ob wir gerade nach HINTEN fallen
        if state != STATE_FALLING and self.falling_threshold_back < self.not_much_smoothed_gyro.get_y() : ###HINTEN
            self.info("ICH FALLE NACH HINTEN ")
            set_state(STATE_FALLING)
            return self.falling_motor_degrees_back

         # Prüfen ob wir gerade nach VORNE fallen
        if state != STATE_FALLING and self.not_much_smoothed_gyro.get_y() < self.falling_threshold_front: ###VORNE
            self.info("ICH FALLE NACH VORNE")
            set_state(STATE_FALLING)
            return self.falling_motor_degrees_front


        #Funktioniert in den meisten Fällen sehr gut!
    cdef check_fallen_sideways(self, set_state ,  state):
        # Prüfen ob wir gerade nach RECHTS fallen
        if state != STATE_FALLING and self.not_much_smoothed_gyro.get_x() < self.falling_threshold_right: ###RECHTS
            self.info("ICH FALLE NACH RECHTS")
            set_state(STATE_FALLING)
            return self.falling_motor_degrees_right

        # Prüfen ob wir gerade nach LINKS fallen
        if state != STATE_FALLING and self.falling_threshold_left < self.not_much_smoothed_gyro.get_x(): ###LINKS
            self.info("ICH FALLE NACH LINKS")
            set_state(STATE_FALLING)
            return self.falling_motor_degrees_left


    cdef get_up(self, set_state):
        """
        This Function handels the Standup procedure
        :return:
        """

        #set_state = <object> #set_state_p
        if self.fallState is FALLEN_BOTTOM_UP:
            # Check if bend forward
            #if self.robo_angle.y > self.config["getting-up-angles"]["bendForward"]["min"] \
            #        and self.robo_angle.y < self.config["getting-up-angles"]["bendForward"]["max"]:
                debug.log("Fallen State: FALLEN_BEND_FORWARD")
                self.fallState = FALLEN_BEND_FORWARD
                return self.config["animations"]["motion"]["toSquat"]
            #else:
            #    #set_state(STATE_FALLEN)
        elif self.fallState is FALLEN_FRONT_UP:
            # Check if bend forward
            #if self.robo_angle.y > self.config["getting-up-angles"]["bendForward"]["min"] \
            #       and self.robo_angle.y < self.config["getting-up-angles"]["bendForward"]["max"]:
                debug.log("Fallen State: FALLEN_BEND_FORWARD")
                self.fallState = FALLEN_BEND_FORWARD
                return self.config["animations"]["motion"]["toSquat"]
            #else:
            #    #set_state(STATE_FALLEN)
        elif self.fallState is FALLEN_BEND_FORWARD:
            # Check if squatted
            #if self.robo_angle.y > self.config["getting-up-angles"]["squatted"]["min"] \
            #        and self.robo_angle.y < self.config["getting-up-angles"]["squatted"]["max"]:
                debug.log("Fallen State: FALLEN_SQUATTED")
                self.fallState = FALLEN_SQUATTED
                return self.config["animations"]["motion"]["stand-up"]
            #else:
            #    #set_state(STATE_FALLEN)
        elif self.fallState is FALLEN_SQUATTED or self.fallState is FALLEN_UPRIGHT:
            # elif self.fallState is FALLEN_SQUATTED:
            # if self.robo_angle.y < 30 and self.robo_angle.y > -10: # TODO: Check value
            # self.animate("walkready")
            # self.fallState = FALLEN_UPRIGHT
            # else:
            # self.state = STATE_FALLEN
            # elif self.fallState is FALLEN_UPRIGHT:
            #if abs(self.robo_angle.y) < 35:  # TODO: Check value
                set_state(STATE_GETTING_UP)
            #else:
            #    #set_state(STATE_FALLEN)
        else:
            # if we are here something went wrong
            set_state(STATE_CONTROLABLE)

    cdef info(self, text):
        debug << text
