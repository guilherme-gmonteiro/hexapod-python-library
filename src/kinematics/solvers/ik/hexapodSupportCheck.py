from ...constants import POSITION_NAME_TO_IS_LEFT_MAP

class HexapodSupportCheck:
    reason = {
        "MIGHT_BE_STABLE_LESS": (
            "Might be stable.\nLess than three known legs are off the ground."
        ),
        "TOO_MANY_LEGS_OFF": (
            "Definitely Unstable.\nToo many legs are off the floor."
        ),
        "RIGHT_LEGS_OFF": (
            "Definitely Unstable.\nAll right legs are off the floor."
        ),
        "LEFT_LEGS_OFF": (
            "Definitely Unstable.\nAll left legs are off the floor."
        ),
        "MIGHT_BE_STABLE_MORE": (
            "Might be stable.\nThree known legs are off the ground.\n"
            "One is on the opposite side of the other two."
        ),
    }

    @staticmethod
    def check_support(legs_names_off_ground):
        reason = HexapodSupportCheck.reason

        if len(legs_names_off_ground) < 3:
            return [False, reason["MIGHT_BE_STABLE_LESS"]]

        if len(legs_names_off_ground) >= 4:
            return [True, reason["TOO_MANY_LEGS_OFF"]]

        # Leg count is exactly 3 at this point
        leg_left_or_right = [
            POSITION_NAME_TO_IS_LEFT_MAP[leg_position]
            for leg_position in legs_names_off_ground
        ]

        if all(is_left is False for is_left in leg_left_or_right):
            return [True, reason["RIGHT_LEGS_OFF"]]

        if all(is_left is True for is_left in leg_left_or_right):
            return [True, reason["LEFT_LEGS_OFF"]]

        return [False, reason["MIGHT_BE_STABLE_MORE"]]