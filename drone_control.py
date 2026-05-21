from mavsdk import System
from mavsdk.offboard import Offboard
from mavsdk.offboard import VelocityNedYaw, PositionNedYaw
import asyncio
import math

class Drone:
    def __init__(self):
        self.drone = System()
        # PID memory for position control
        self.prev_error_north = 0.0
        self.prev_error_east = 0.0
        self.prev_error_down = 0.0

        self.integral_error_north = 0.0
        self.integral_error_east = 0.0
        self.integral_error_down = 0.0

        self.prev_error_yaw = 0.0
        self.integral_error_yaw = 0.0

        self.last_time = None

    def _normalize_yaw(self, yaw_deg):
        while yaw_deg > 180:
            yaw_deg -= 360
        while yaw_deg < -180:
            yaw_deg += 360
        return yaw_deg

    def _yaw_error(self, target, current):
        error = target - current
        while error > 180:
            error -= 360
        while error < -180:
            error += 360
        return error

    async def connect(self):
        await self.drone.connect(system_address="udpin://0.0.0.0:14540")

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("Connected")
                break

    async def arm_and_takeoff(self):
        await self.drone.action.arm()
        await self.drone.action.takeoff()
        await asyncio.sleep(20)
        print("Takeoff")
 # Required before start
        await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0.0, 0.0, 0.0, 0.0))
        # start offboard mode
        await self.drone.offboard.start()

    async def land(self):
        await self.drone.offboard.stop()
        await self.drone.action.land()
        await asyncio.sleep(10)
        print("land")
        await self.drone.action.disarm()

    async def get_position(self):
        async for pos in self.drone.telemetry.position_velocity_ned():
            return pos.position.north_m, pos.position.east_m, pos.position.down_m

    async def get_yaw(self):
        async for att in self.drone.telemetry.attitude_euler():
            return att.yaw_deg
        
    async def get_velo(self):
        async for velo in self.drone.telemetry.position_velocity_ned():
            return velo.velocity.north_m_s, velo.velocity.east_m_s, velo.velocity.down_m_s

    async def send_velocity(self, vx, vy, vz,yaw_deg):
         await self.drone.offboard.set_velocity_ned(VelocityNedYaw(north_m_s=vx, east_m_s=vy, down_m_s=vz, yaw_deg=yaw_deg))

    def reset_position_pid(self):
        self.prev_error_north = 0.0
        self.prev_error_east = 0.0
        self.prev_error_down = 0.0
        self.prev_error_yaw = 0.0

        self.integral_error_north = 0.0
        self.integral_error_east = 0.0
        self.integral_error_down = 0.0
        self.integral_error_yaw = 0.0

        self.last_time = None

    async def custom_position_setpoint(self, north, east, down, yaw_deg):
        # Reset PID memory between setpoints
        self.reset_position_pid()
        # PID constants
        Kp_velo = 2.5
        Ki_velo = 0.0
        Kd_velo = 1.9

        I_max = 0.0
        max_velo = 20

        Kp_down = 3.0
        Ki_down = 1.0
        Kd_down = 1.5

        I_max_down = 2.0
        max_down_velo = 10

        Kp_yaw = 2.0
        Ki_yaw = 0.0
        Kd_yaw = 4.0
        max_yaw_velo = 90

        I_max_yaw = 0.0

        while not await self.is_position_reached(north, east, down, yaw_deg):
            curr_pos_n, curr_pos_e, curr_pos_d = await self.get_position()
            curr_yaw = await self.get_yaw()
            curr_velo_n, curr_velo_e, curr_velo_d = await self.get_velo()

            # =========================
            # NORTH PID
            # =========================
            n_error = north - curr_pos_n
            n_P = n_error * Kp_velo
            n_D = -curr_velo_n * Kd_velo
            self.integral_error_north += n_error
            if self.integral_error_north > I_max:
                self.integral_error_north = I_max
            elif self.integral_error_north < -I_max:
                self.integral_error_north = -I_max
            n_I = self.integral_error_north * Ki_velo
            vx = n_P + n_D + n_I
            if vx > max_velo:
                vx = max_velo
            elif vx < -max_velo:
                vx = -max_velo
            self.prev_error_north = n_error

            # =========================
            # EAST PID
            # =========================
            e_error = east - curr_pos_e
            e_P = e_error * Kp_velo
            e_D = -curr_velo_e * Kd_velo
            self.integral_error_east += e_error
            if self.integral_error_east > I_max:
                self.integral_error_east = I_max
            elif self.integral_error_east < -I_max:
                self.integral_error_east = -I_max
            e_I = self.integral_error_east * Ki_velo
            vy = e_P + e_D + e_I
            if vy > max_velo:
                vy = max_velo
            elif vy < -max_velo:
                vy = -max_velo
            self.prev_error_east = e_error

            # =========================
            # DOWN PID
            # =========================
            d_error = down - curr_pos_d
            d_P = d_error * Kp_down
            d_D = -curr_velo_d * Kd_down
            self.integral_error_down += d_error
            if self.integral_error_down > I_max_down:
                self.integral_error_down = I_max_down
            elif self.integral_error_down < -I_max_down:
                self.integral_error_down = -I_max_down
            d_I = self.integral_error_down * Ki_down
            vz = d_P + d_D + d_I
            if vz > max_down_velo:
                vz = max_down_velo
            elif vz < -max_down_velo:
                vz = -max_down_velo
            self.prev_error_down = d_error

            # =========================
            # YAW PID
            # =========================
            yaw_error = self._yaw_error(yaw_deg, curr_yaw)
            yaw_P = yaw_error * Kp_yaw
            yaw_D = (yaw_error - self.prev_error_yaw) * Kd_yaw
            self.integral_error_yaw += yaw_error
            if self.integral_error_yaw > I_max_yaw:
                self.integral_error_yaw = I_max_yaw
            elif self.integral_error_yaw < -I_max_yaw:
                self.integral_error_yaw = -I_max_yaw
            yaw_I = self.integral_error_yaw * Ki_yaw
            yaw_output = yaw_P + yaw_D + yaw_I
            if yaw_output > max_yaw_velo:
                yaw_output = max_yaw_velo
            elif yaw_output < -max_yaw_velo:
                yaw_output = -max_yaw_velo
            yaw_step = self._normalize_yaw(curr_yaw + yaw_output)
            self.prev_error_yaw = yaw_error

            await self.send_velocity(vx,vy,vz,yaw_step)
            await asyncio.sleep(0.05)

    async def send_position_setpoint(self, north, east, down, yaw_deg):
        await self.drone.offboard.set_position_ned(PositionNedYaw(north_m=north, east_m=east, down_m=down, yaw_deg=yaw_deg))

    async def rotate_to_yaw(self, target_yaw_deg, tolerance=2.0):
        """
        Rotate to a target yaw using PID control
        """
        target_yaw_deg = self._normalize_yaw(target_yaw_deg)

        # PID gains (tune these!)
        Kp = 0.8
        Ki = 0.0
        Kd = 0.2

        integral = 0.0
        prev_error = 0.0

        dt = 0.1  # 10 Hz loop

        while True:
#            yaw_rad = await self.get_yaw()
            current_yaw = await self.get_yaw()

            error = self._yaw_error(target_yaw_deg, current_yaw)

            # Stop condition
            if abs(error) < tolerance:
                break

            # PID terms
            integral += error * dt
            derivative = (error - prev_error) / dt

            output = Kp * error + Ki * integral + Kd * derivative

            # Clamp yaw rate (deg/s equivalent behavior)
            max_yaw_rate = 60.0
            output = max(min(output, max_yaw_rate), -max_yaw_rate)

            # Convert to target yaw step
            new_yaw = current_yaw + output * dt
            new_yaw = self._normalize_yaw(new_yaw)

            # Send command
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(
                    north_m_s=0.0,
                    east_m_s=0.0,
                    down_m_s=0.0,
                    yaw_deg=new_yaw
                )
            )

            prev_error = error
            await asyncio.sleep(dt)

        # Final stabilization
        await self.drone.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, target_yaw_deg)
        )

    # =========================
    # 🚁 HIGH-LEVEL COMMANDS
    # =========================

    async def turn_cw_90(self):
        current = await self.get_yaw()
        await self.rotate_to_yaw(current + 90)

    async def turn_ccw_90(self):
        current = await self.get_yaw()
        await self.rotate_to_yaw(current - 90)

    async def turn_cw_180(self):
        current = await self.get_yaw()
        await self.rotate_to_yaw(current + 180)

    async def is_position_reached(
    self,
    target_north,
    target_east,
    target_down,
    target_yaw, # degrees
    pos_tolerance=0.15, # degrees
    yaw_tolerance=10.0, # degrees
    ):
        north, east, down = await self.get_position()
        
        # Calc euclidean distance
        distance_error = math.sqrt(
            (target_north - north) ** 2 +
            (target_east - east) ** 2 +
            (target_down - down) ** 2
        )

        yaw = await self.get_yaw()
        # Calc yaw error
        yaw_error = abs(yaw - target_yaw)

        if distance_error < pos_tolerance and yaw_error < yaw_tolerance:
            print(f"Reached target, error={distance_error:.2f} m")
            return True
        else:
            return False

    async def wait_until_position_reached(
    self,
    target_north,
    target_east,
    target_down,
    target_yaw, # degrees
    pos_tolerance=0.1, # degrees
    yaw_tolerance=5.0, # degrees
    ):

        while True:
            north, east, down = await self.get_position()
            
            # Calc euclidean distance
            distance_error = math.sqrt(
                (target_north - north) ** 2 +
                (target_east - east) ** 2 +
                (target_down - down) ** 2
            )

            yaw = await self.get_yaw()
            # Calc yaw error
            yaw_error = abs(yaw - target_yaw)

            if distance_error < pos_tolerance and yaw_error < yaw_tolerance:
                print(f"Reached target, error={distance_error:.2f} m")
                return True

            await asyncio.sleep(0.1)