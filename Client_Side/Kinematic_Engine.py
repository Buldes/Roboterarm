import math
import time


class Kinematik_Engine:
    def __init__(self,
                 joints_intervall: tuple[int] = (0, 180),
                 base_step_winkel: float = 0.6,
                 total_length: float = 30.0
                 ):

        # Limit Value
        self.joints_intervall: tuple[int] = joints_intervall
        self.base_step_winkel: float = base_step_winkel
        self.total_length: float = total_length
        self.max_length: float = 0.91

        # calculating process
        self.valid_angle: bool = None
        self.starting_angle: float = 0.0
        self.angle_change: float = 0.01


    def calc_angle_by_normalized(self, x, y, z, use_specific_angle: float = -1):
        self.valid_angle = False
        self.starting_angle: float = 0

        valid_length_z_factor: bool = False

        z_vector = (x, y, z)
        z_vector_2D = None

        base_rotation: float = 0.0
        joint_rotation: list[float] = [0.0, 0.0, 0.0]
        vector_length: float = 1.0 / 3.0

        if use_specific_angle > -1:
            self.starting_angle = use_specific_angle

        """4.2.X"""
        # calculate join rotation
        while not self.valid_angle:
            try:
                """5.1"""
                if not valid_length_z_factor:

                    z_vector = self.check_length_z_factor(z_vector)
                    valid_length_z_factor = True

                """4.2.1"""
                # convert 3D Vector into 2D Vector
                z_vector_2D = (
                    math.sqrt(z_vector[0] ** 2 + z_vector[2] ** 2),
                    z_vector[1]
                )

                """4.2.3"""
                # calculate vector p_3
                g_3 = self.calc_vector_by_angle(self.starting_angle)
                p_3 = (
                    z_vector_2D[0] - g_3[0],
                    z_vector_2D[1] - g_3[1]
                )

                """4.2.3"""
                # calculate p_3 length
                p_3_length = math.sqrt(p_3[0] ** 2 + p_3[1] ** 2)

                """5.2.2"""
                # check p_3 length
                if p_3_length > 2 * vector_length + 0.000001:
                    if self.starting_angle >= 360 or use_specific_angle > -1:
                        return False

                    self.starting_angle += self.angle_change
                    continue

                """4.2.4"""
                # calculate joint 2 angle
                p_3_length = min(p_3_length, vector_length * 2) # float error correction

                rot = math.acos(
                    (vector_length ** 2 + vector_length ** 2 - p_3_length ** 2) /
                    (2 * vector_length * vector_length)
                )

                final_rot = 180 - math.degrees(rot)
                joint_rotation[1] = final_rot

                """4.2.5"""
                # calculate joint 1 angle
                angle_y = (180 - math.degrees(rot)) / 2
                angle_e = math.atan2(p_3[1], p_3[0])
                final_rot = 90 - (angle_y + math.degrees(angle_e))
                joint_rotation[0] = final_rot

                """4.2.6"""
                # calculate joint 3 angle
                final_rot = self.starting_angle + 90 - (joint_rotation[0] + joint_rotation[1])
                joint_rotation[2] = final_rot % 360

                """5.2.1"""
                if self.check_valid_joints_angles(joint_rotation):
                    break

                """5.2.3"""
                self.starting_angle += self.angle_change
                if self.starting_angle >= 360 or use_specific_angle > -1:
                    return False

            except Exception as e:
                return False

        """4.1.X"""
        base_rotation = self.cal_base_rotation(z_vector)

        return [base_rotation, joint_rotation]

    def check_valid_joints_angles(self, joints):
        if not 0 <= joints[0] <= 90:
            return False
        elif not self.joints_intervall[0] <= joints[1] <= self.joints_intervall[1]:
            return False
        elif not self.joints_intervall[0] <= joints[2] <= self.joints_intervall[1]:
            return False
        return True

    def cal_base_rotation(self, vec):
        # calculate base rotation
        base_rotation_degree = math.atan2(vec[2], vec[0]) * (180 / math.pi)
        base_rotation_intervall = (base_rotation_degree + 360) % 360
        rest_rotation = base_rotation_intervall % self.base_step_winkel
        if rest_rotation < self.base_step_winkel / 2:
            final_base_rotation = base_rotation_intervall - rest_rotation
        else:
            final_base_rotation = base_rotation_intervall + (self.base_step_winkel - rest_rotation)

        return final_base_rotation

    def check_length_z_factor(self, vec):
        # check if length is valid
        length = math.sqrt(vec[0]**2 + vec[1]**2 + vec[2]**2)

        # calc scale factor
        scale_factor = self.max_length / length
        # set to max of self.max_length % (Default 92%)
        scale_factor = min(scale_factor, self.max_length)
        # calc new vec
        new_vec = (
            vec[0] * scale_factor,
            vec[1] * scale_factor,
            vec[2] * scale_factor
        )
        return new_vec

    def calc_angle_by_real(self, x, y, z, use_specific_angle: float = -1):
           return self.calc_angle_by_normalized(
               self.real_to_normalized(x),
               self.real_to_normalized(y),
               self.real_to_normalized(z),
               use_specific_angle
        )

    def real_to_normalized(self, value):
        return value / self.total_length

    def normalized_to_real(self, value):
        return value * self.total_length

    def calc_vector_by_angle(self, angle):
        relative_angle = math.radians(angle + 90)
        return (
            (1/3) * math.sin(relative_angle),
            (1/3) * math.cos(relative_angle)
        )

    def performance_test(self):

        start = time.time()

        result = self.calc_angle_by_normalized(1, 1, 1)

        print(f"\nResult 1: {result} \nin Time: {time.time() - start:.2f}\nRunning Test 2...")


        start = time.time()

        result = self.calc_angle_by_normalized(0.2, 1, 1)

        print(f"\nResult 2: {result} \nin Time: {time.time() - start:.2f}\n")


if __name__ == '__main__':
    ke = Kinematik_Engine()

    ke.performance_test()


