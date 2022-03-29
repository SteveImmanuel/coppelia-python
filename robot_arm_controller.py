import connector.sim as sim
import time
import cv2
import numpy as np

movementSpeed = 0.025
rotationSpeed = 0.5

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

#Connect to the simulation
if clientID != -1:
    currentTime = time.time()

    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking)
    print('Connected to remote API server')

    #Get handles to simulation objects
    print('Obtaining handles of simulation objects')

    res, camera = sim.simxGetObjectHandle(clientID, 'VisionSensor', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok: print('Could not get handle to Camera')
    res, targetObj = sim.simxGetObjectHandle(clientID, 'sphere', sim.simx_opmode_oneshot_wait)
    if res != sim.simx_return_ok: print('Could not get handle to TargetObj for IK')

    #Start main control loop
    print('Starting control loop')
    res, resolution, image = sim.simxGetVisionSensorImage(clientID, camera, 0, sim.simx_opmode_streaming)
    res, pos = sim.simxGetObjectPosition(clientID, camera, -1, sim.simx_opmode_oneshot_wait)
    res, rot = sim.simxGetObjectOrientation(clientID, camera, -1, sim.simx_opmode_oneshot_wait)

    while sim.simxGetConnectionId(clientID) != -1:
        lastTime = currentTime
        currentTime = time.time()
        deltaTime = currentTime - lastTime

        # res, pos = sim.simxGetObjectPosition(clientID, camera, -1, sim.simx_opmode_buffer)
        # res, rot = sim.simxGetObjectOrientation(clientID, camera, -1, sim.simx_opmode_buffer)
        res, resolution, image = sim.simxGetVisionSensorImage(clientID, camera, 0, sim.simx_opmode_buffer)

        #Get image from Camera
        if res == sim.simx_return_ok:
            original = np.array(image, dtype=np.uint8)
            original.resize([resolution[0], resolution[1], 3])
            original = cv2.flip(original, 0)
            original = cv2.cvtColor(original, cv2.COLOR_RGB2BGR)
            cv2.imshow('Camera', original)

        keypress = cv2.waitKey(1) & 0xFF  #will read a value of 255 if no key is pressed
        # print(pos)
        if keypress == ord('d'):
            pos[0] += movementSpeed * deltaTime
        elif keypress == ord('a'):
            pos[0] -= movementSpeed * deltaTime
        elif keypress == ord('s'):
            pos[1] -= movementSpeed * deltaTime
        elif keypress == ord('w'):
            pos[1] += movementSpeed * deltaTime
        elif keypress == ord('q'):
            pos[2] -= movementSpeed * deltaTime
        elif keypress == ord('e'):
            pos[2] += movementSpeed * deltaTime
        elif keypress == ord('t'):
            rot[0] += rotationSpeed * deltaTime
        elif keypress == ord('g'):
            rot[0] -= rotationSpeed * deltaTime
        elif keypress == ord('y'):
            rot[1] += rotationSpeed * deltaTime
        elif keypress == ord('h'):
            rot[1] -= rotationSpeed * deltaTime
        elif keypress == ord('u'):
            rot[2] += rotationSpeed * deltaTime
        elif keypress == ord('j'):
            rot[2] -= rotationSpeed * deltaTime

        sim.simxSetObjectPosition(clientID, targetObj, -1, pos, sim.simx_opmode_oneshot)
        sim.simxSetObjectOrientation(clientID, targetObj, -1, rot, sim.simx_opmode_oneshot)

else:
    print('Could not connect to remote API server')

#Close all simulation elements
sim.simxStopSimulation(clientID, sim.simx_opmode_blocking)
sim.simxFinish(clientID)
cv2.destroyAllWindows()
print('Simulation ended')
