#!/usr/bin/env python3
import rospy

import cv2
import os
import time

from grid import find_game_grid, find_color, debug_image
import setting
from MinMaxSolver import MinMaxSolver
from LetterRecognition import LetterRecognition
from corrado_camera.msg import BestMove

def main():
    rospy.init_node("corrado_camera_node", anonymous=True)
    best_move_pub = rospy.Publisher('best_move', BestMove, queue_size=1)
    best_move_msg = BestMove()

    # Inizializza la webcam
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        rospy.logerr("Impossibile inizializzare camera!")
    cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
    cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("threshold", "mask", 120, 255, lambda x: None)
    cv2.createTrackbar("kernel_dim", "mask", 7, 15, lambda x: None)

    # Crea un'istanza della classe con il percorso dei pesi del modello
    current_path = os.path.dirname(__file__)
    relative_path = 'weights.best.xo.hdf5'
    full_path = os.path.join(current_path, relative_path)
    letter_recog = LetterRecognition(full_path)

    # Gestione campionamento
    rate = rospy.Rate(10) #Hz
    acc_score = 0
    prec_config = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    solver = MinMaxSolver()
    solved = False

    while not rospy.is_shutdown():

        config = [0, 0, 0, 0, 0, 0, 0, 0, 0]

        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Impossibile leggere immagine dalla camera!")
        frame = cv2.resize(frame, (1280, 960))
        display_frame = frame.copy()
        threshold = cv2.getTrackbarPos("threshold", "mask")
        kernel_dim = cv2.getTrackbarPos("kernel_dim", "mask")

        try:
            # Trova la griglia
            grid_frame, thresholded, corners = find_game_grid(frame, threshold, kernel_dim)
            # Disegno ROI e rilevazione griglia
            for index, corner in enumerate(corners):
                cv2.rectangle(display_frame, (corner[0][0]+setting.roi_x, corner[0][1]+setting.roi_y),
                              (corner[1][0]+setting.roi_x, corner[1][1]+setting.roi_y), (0, 255, 0), thickness=2)
                cv2.putText(display_frame, str(index), (corner[0][0]+setting.roi_x, corner[0][1]+setting.roi_y),
                            cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.5, color=(0, 0, 255), thickness=1)

            for index, cell in enumerate(corners):
                extracted_frame = grid_frame[cell[0][1]:cell[1][1], cell[0][0]:cell[1][0]]
                (h, w) = extracted_frame.shape[:2]
                if find_color(extracted_frame):
                    # Utilizza la classe per riconoscere le lettere
                    recognized_letter = letter_recog.recognize_letter(cv2.resize(extracted_frame,(32,32)))
                    if recognized_letter == 'х':
                        config[index] = setting.X_SYM
                        cv2.line(display_frame, (setting.roi_x+cell[0][0]+10, setting.roi_y+cell[0][1]+10),
                                 (setting.roi_x+cell[1][0]-10, setting.roi_y+cell[1][1]+-10), (0, 0, 255), 2)
                        cv2.line(display_frame, (setting.roi_x+cell[0][0]+w-10, setting.roi_y+cell[0][1]+10),
                                 (setting.roi_x+cell[1][0]-w+10, setting.roi_y+cell[1][1]-10), (0, 0, 255), 2)
                    elif recognized_letter == 'о':
                        config[index] = setting.O_SYM
                        cv2.circle(display_frame, (setting.roi_x+cell[0][0]+w//2, setting.roi_y+cell[0][1]+h//2),
                                   22, (0, 0, 255), 2)
                else:
                    config[index] = 0
        except TypeError:
            # rospy.logerr("errore")
            pass

        cv2.rectangle(display_frame, (setting.roi_x, setting.roi_y), (setting.roi_x+setting.roi_width, setting.roi_y+setting.roi_height), color=(255, 0, 0), thickness=2)


        # Assicuriamoci dell'accuratezza della rilevazione
        if config == prec_config:
            acc_score += 1
            # Stampo la mossa migliore
            if acc_score > 5 and not solved:
                solver.set_config(config)
                best_move = solver.find_best_move()
                solved = True
                best_move_msg.header.stamp = rospy.Time.now()
                best_move_msg.mossa = best_move
                best_move_msg.score = acc_score
                best_move_pub.publish(best_move_msg)
            if solved:
                try:
                    cv2.line(display_frame, (
                    setting.roi_x + corners[best_move][0][0] + 10, setting.roi_y + corners[best_move][0][1] + 10),
                             (setting.roi_x + corners[best_move][1][0] - 10,
                              setting.roi_y + corners[best_move][1][1] + -10), (255, 0,), 2)
                    cv2.line(display_frame, (
                    setting.roi_x + corners[best_move][0][0] + w - 10, setting.roi_y + corners[best_move][0][1] + 10),
                             (setting.roi_x + corners[best_move][1][0] - w + 10,
                              setting.roi_y + corners[best_move][1][1] - 10), (255, 0,), 2)
                except IndexError:
                    pass
        else:
            acc_score = 0
            solved = False
            best_move = None
        prec_config = config


        # Mostra il frame
        cv2.imshow("frame", display_frame)
        # DEBUG calibrazione threshold
        cv2.imshow("mask", thresholded)
        debug_image(grid_frame)

        if cv2.waitKey(1) & 0xFF == 27:  # Esc per uscire
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass