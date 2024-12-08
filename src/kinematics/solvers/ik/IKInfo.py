class IKMessage:
    success = {
        "subject": "Success.",
        "body": "All legs are on the floor.",
    }

    @staticmethod
    def success_legs_on_air(legs):
        return {
            "subject": "Success.",
            "body": (
                f"But some legs won't reach target points on the ground:\n"
                f"{IKMessage.bullet_points(legs)}"
            ),
        }

    @staticmethod
    def no_support(reason, legs, flags=None):
        if flags is None:
            flags = {"listLegs": False}
        return {
            "subject": "Failure: No Support.",
            "body": (
                f"{reason}\n"
                f"{IKMessage.bullet_points(legs) if flags.get('listLegs') else ''}"
            ),
        }

    @staticmethod
    def bad_point(point):
        return {
            "subject": "Failure: Bad Point.",
            "body": (
                "At least one point would be shoved to the ground:\n"
                f"{point.to_markdown_string()}"
            ),
        }

    @staticmethod
    def bullet_points(elements):
        return "".join(f" - {position}\n" for position in elements)

    @staticmethod
    def bad_leg(message):
        return {
            "subject": "Failure: Bad leg.",
            "body": message,
        }

    @staticmethod
    def alpha_not_in_range(position, alpha, max_angle):
        return {
            "subject": "Failure: Alpha not within range",
            "body": (
                f"The alpha ({alpha}) computed for {position} leg "
                f"is not within -{max_angle} < alpha < {max_angle}"
            ),
        }

    initialized = {
        "subject": "Initialized",
        "body": "Has not solved for anything yet.",
    }


class LegIKInfo:
    @staticmethod
    def target_reached(position):
        return {
            "legPosition": position,
            "message": f"Success! ({position})",
            "obtainedSolution": True,
            "reachedTarget": True,
        }

    @staticmethod
    def target_not_reached(position):
        return {
            "legPosition": position,
            "message": f"Success! But this leg won't reach the target ground point. ({position})",
            "obtainedSolution": True,
            "reachedTarget": False,
        }

    @staticmethod
    def blocked(position):
        return {
            "legPosition": position,
            "message": (
                f"Failure. The ground is blocking the path. The target point can only be reached "
                f"by digging the ground. ({position})"
            ),
            "obtainedSolution": False,
            "reachedTarget": True,
        }

    @staticmethod
    def femur_too_long(position):
        return {
            "legPosition": position,
            "message": f"Failure. Femur length too long. ({position})",
            "obtainedSolution": False,
            "reachedTarget": False,
        }

    @staticmethod
    def tibia_too_long(position):
        return {
            "legPosition": position,
            "message": f"Failure. Tibia length too long. ({position})",
            "obtainedSolution": False,
            "reachedTarget": False,
        }

    @staticmethod
    def initialized(position):
        return {
            "legPosition": position,
            "obtainedSolution": False,
            "reachedTarget": False,
            "message": f"Haven't solved anything yet. ({position})",
        }