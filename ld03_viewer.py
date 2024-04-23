import serial, time, sys
import pygame, math

pygame.init()


monitorInfo = pygame.display.Info()
initialSize = min(monitorInfo.current_w, monitorInfo.current_h)*0.9
WS = (initialSize, initialSize)

# screen drawing config
maxDist = 3500 # maximum distance (in mm) covered by the polar graph
myfont = pygame.freetype.SysFont('Consolas', 18)

# polar grid config
gridDistStep = 500
gridAngleStep = 30
gridColor = (50,50,50)

# Speed control config
speed = 0
percentOn = 100.0
targetSpeed = 3000
controlSpeed = True

previousTime = 0


# this CRC calc is a simplified version of https://gist.github.com/Lauszus/6c787a3bc26fea6e842dfb8296ebd630
def crc8_poly(data, poly, crc=0, xor_out=0):
    g = 0x100 | poly  # Generator polynomial
    # Loop over the data
    for d in data:
        # XOR the top byte in the CRC with the input byte
        crc ^= d 
        # Loop over all the bits in the byte
        for _ in range(8):
            # Start by shifting the CRC, so we can check for the top bit
            crc <<= 1
            # XOR the CRC if the top bit is 1
            if crc & 0x100:
                crc ^= g
    # Return the CRC value
    return crc ^ xor_out


def drawScreen(window, points):
    global previousTime, speed, gridDistStep, gridAngleStep
    # t1 = time.perf_counter(), time.process_time()
    
    actualTime = time.time()
    timeDelta = actualTime - previousTime
    previousTime = actualTime
    fps = 1/timeDelta
    
    window.fill((0,0,0))
    center = tuple(i/2 for i in WS)
    radius = (min(WS)/2)*0.9 # 90% 
    
    # draw polar grid
    for griddist in range(gridDistStep, maxDist, gridDistStep):
        pygame.draw.circle(window, gridColor, center=center, radius=(griddist/maxDist * radius), width=1)
    
    for gridangle in range(0, 360, gridAngleStep):
        pygame.draw.aaline(window, gridColor, start_pos=center, end_pos=(math.cos(math.radians(gridangle))*radius + center[0], math.sin(math.radians(gridangle))*radius + center[1]) )
    
    # draw border and center
    pygame.draw.circle(window,(0,200,0), center=center, radius=radius, width=1) # border
    pygame.draw.circle(window,(255,0,0), center=center, radius=3, width=0)
    
    if controlSpeed:
        myfont.render_to(window, (10, 10), f"Speed: Target {targetSpeed}º/s Actual {speed}º/s Duty {percentOn:.2f}%", (255, 255, 255))
    myfont.render_to(window, (10, 30), f"FPS {fps:.2f} points {len(points)}", (255, 255, 255))
    
    myfont.render_to(window, (10, WS[1]-18), f"Graph's radius: {maxDist}mm grid: {gridDistStep}mm {gridAngleStep}º", (255, 255, 255))
    
    # allows auto adjusting the distance
    # maxDist = max([point['distance'] for point in points ])

    for point in points:
        distanceInPixels = (point['distance']/maxDist * radius)
        distanceInPixels = min(distanceInPixels, radius) # draw in the border if it's further away
        ScreenPoint = (math.cos(math.radians(point['angle']))*distanceInPixels + center[0], math.sin(math.radians(point['angle']))*distanceInPixels + center[1])
        # pointcolor = (200,200,0)
        pointcolor = (point['intensity'], point['intensity'], point['intensity'])
        pygame.draw.circle(window, pointcolor, center=ScreenPoint, radius=2, width=0)
    pygame.display.update()
    # t2 = time.perf_counter(), time.process_time()
    # print(f" Real time: {(t2[0] - t1[0])*1000:.2f} ms  CPU time: {(t2[1] - t1[1])*1000:.2f} ms")


if __name__ == "__main__":
    
    if (len(sys.argv) < 2):
        print("Please specify a serial port")
        exit()
    serialPort = sys.argv[1]
    ser = serial.Serial(serialPort, 115200, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE, timeout=1 )
    ser.flush()

    existing_points = []
    
    if controlSpeed:
        print("Sending initial zeros")
        # sends a initial stream of zeros to decelerate the motor in case it is too fast to report
        ser.write(bytearray([0]*5000)) 
    
    print("Starting loop")
    
    window = pygame.display.set_mode(WS, pygame.RESIZABLE)
    pygame.display.set_caption("LD03 viewer")


    active = True
    while active:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                active = False
            if event.type == pygame.VIDEORESIZE:
                    WS = event.size
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    targetSpeed +=50
                if event.key == pygame.K_DOWN:
                    targetSpeed -=50
                if event.key == pygame.K_RIGHT:
                    maxDist +=100
                if event.key == pygame.K_LEFT:
                    maxDist -=100
                if event.key == pygame.K_SPACE:
                    controlSpeed = not controlSpeed
                    percentOn = 100.0
                
                
        # TODO run the serial RX on another Thread to avoid loss of packets due to UI processing
        # wait for the header
        receivedHeader = None
        while (receivedHeader != b'\x54\x2C'):
            receivedHeader = ser.read(2)
        
        # now receive the body of the packet
        body = ser.read(45)
        # print(' '.join(f'{x:02x}' for x in body))
        
        # Validate the data with CRC before accepting the packet
        # CRC poly extracted with reveng:
        # width=8  poly=0x4d  init=0x00  refin=false  refout=false  xorout=0x00  check=0xc3  residue=0x00  name=(none)
        
        receivedCRC = body[-1]
        
        fullmsgWithoutCRC = receivedHeader + body[:-1]
        
        calculatedCRC = crc8_poly(fullmsgWithoutCRC, 0x4d)
        
        if (calculatedCRC != receivedCRC):
            print(f"CRC did not match {receivedCRC} <> {calculatedCRC}")
            continue
        
        # extract the information from the packet
        speed = int.from_bytes(body[0:2], "little")
        start_angle = int.from_bytes(body[2:4], "little")/100 # according to protocol is 0.01º
        dataPayload = body[4:40]
        end_angle = int.from_bytes(body[40:42], "little")/100
        
        if (end_angle < start_angle):
            # wrapped 
            end_angle += 360.0
        step_angle = (end_angle - start_angle)/11
        
        # decode the datapoints
        for index in range(12):
            arrayindex = index*3
            distance = int.from_bytes(dataPayload[(arrayindex):(arrayindex+2)], "little")
            intensity = int(dataPayload[arrayindex+2])
            if (distance and intensity):
                angle = start_angle + step_angle*index
                angle %= 360.0 # wrap around 360
                existing_points.append({'angle': angle, 'distance': distance, 'intensity': intensity })
        
        if (end_angle >= 360.0):
            # Draw a new frame every full revolution
            drawScreen(window, existing_points)
            existing_points = [] # clear the points 
            
        if controlSpeed:
            # very crude integrator for speed control
            # TODO implement a proper (PI?) control loop
            error = (targetSpeed - speed)/50000
            percentOn += error
            percentOn = min(100.0, max(45.0, percentOn))
            
            # emulate a crude PWM signal transmitting sequences of 
            # 0x00 over serial, effectively decreasing the RMS value
            values = bytearray([0]*int(100-percentOn))
            ser.write(values)
            # print(f'{speed} {percentOn:.2f}')
        
    
    ser.close()
