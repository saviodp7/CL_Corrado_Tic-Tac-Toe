#!/usr/bin/env python3
import rospy

import cv2
import os

from grid import find_game_grid, print_board, find_color
import setting
from MinMaxSolver import MinMaxSolver
from LetterRecognition import LetterRecognition
from corrado_camera.msg import BestMove

def main():
    rospy.init_node("corrado_camera_node", anonymous=True)
    best_move_pub = rospy.Publisher('best_move', BestMove, queue_size=1)
    best_move_msg = BestMove()

    # Inizializza la webcam
    cap = cv2.VideoCapture(0)
    cv2.namedWindow("config", cv2.WINDOW_NORMAL)
    cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
    cv2.createTrackbar("threshold", "mask", 77, 255, lambda x: None)
    cv2.createTrackbar("kernel_dim", "mask", 10, 15, lambda x: None)

    # Crea un'istanza della classe con il percorso dei pesi del modello
    current_path = os.path.dirname(__file__)
    relative_path = 'weights.best.xo.hdf5'
    full_path = os.path.join(current_path, relative_path)
    letter_recog = LetterRecognition(full_path)

    # Gestione campionamento
    rate = rospy.Rate(5) # 2Hz
    acc_score = 0
    prec_config = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    solver = MinMaxSolver()

    while not rospy.is_shutdown():

        config = [0, 0, 0, 0, 0, 0, 0, 0, 0]
        ret, frame = cap.read()
        cv2.rectangle(frame, (setting.roi_x, setting.roi_y), (setting.roi_x+setting.roi_width, setting.roi_y+setting.roi_height), color=(255, 0, 0), thickness=2)
        threshold = cv2.getTrackbarPos("threshold", "mask")
        kernel_dim = cv2.getTrackbarPos("kernel_dim", "mask")

        try:
            # Trova e disegna la griglia del tris
            grid_frame, thresholded, corners = find_game_grid(frame, threshold, kernel_dim)
            for index, cell in enumerate(corners):
                extracted_frame = grid_frame[cell[0][1]:cell[1][1], cell[0][0]:cell[1][0]]
                (h, w) = extracted_frame.shape[:2]
                cv2.imwrite("frame_estratto.jpg", extracted_frame)
                if find_color(extracted_frame):
                    # Utilizza la classe per riconoscere le lettere
                    recognized_letter = letter_recog.recognize_letter("frame_estratto.jpg")
                    if recognized_letter == 'х':
                        config[index] = setting.X_SYM
                        cv2.line(extracted_frame, (10, 10), (h-10, h-10), (0, 0, 255), 2)
                        cv2.line(extracted_frame, (10, h-10), (h-10, 10), (0, 0, 255), 2)
                    elif recognized_letter == 'о':
                        config[index] = setting.O_SYM
                        cv2.circle(extracted_frame, (w//2, h//2), 22, (0, 0, 255), 2)
                else:
                    config[index] = 0
        except TypeError:
            pass

        # Mostra il frame elaborato
        cv2.imshow("config", frame)
        # DEBUG
        cv2.imshow("mask", thresholded)

        # Assicuriamoci dell'accuratezza della rilevazione
        if config == prec_config:
            acc_score += 1
        else:
            acc_score = 0
        prec_config = config

        # Stampo la mossa migliore
        if acc_score > 10:
            solver.set_config(config)
            best_move = solver.find_best_move()
            best_move_msg.header.stamp = rospy.Time.now()
            best_move_msg.mossa = best_move
            best_move_msg.score = acc_score
            best_move_pub.publish(best_move_msg)




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
