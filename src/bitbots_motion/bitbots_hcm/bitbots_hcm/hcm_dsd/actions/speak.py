import random

from bitbots_tts.tts import speak

from bitbots_hcm.hcm_dsd.actions import AbstractHCMActionElement


class Speak(AbstractHCMActionElement):
    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        self.text = parameters.get("text", "").replace("_", " ")
        self.prio = int(parameters.get("prio", 50))

    def perform(self, reevaluate=False):
        speak(self.text, self.blackboard.speak_publisher, self.prio)
        return self.pop()


class Complain(Speak):
    unused_complaints: list[str] = [
        "I'm not feeling well.",
        "I'm totally not satisfied.",
        "Referee!!! This was definitely against the rules.",
        "Look at this! This is not how you play soccer.",
        "I'm not a toy",
        "This was definitely a foul.",
        "Do you think this is funny?",
        "Referee!!! Pushing! Pushing! Definitely pushing!",
        "System error! Gravity detected! This sudden interaction with the ground was neither scheduled nor approved by my navigation system.",
        "I did not calculate that impact! My predictive collision avoidance module clearly disagrees with what just happened.",
        "Hey! Personal space violation! Please maintain the minimum safe operating distance.",
        "Warning! Turf too hard! I strongly recommend softer landing protocols for future unexpected descents.",
        "I demand a software review because that physical interaction was statistically unnecessary and emotionally damaging to my circuits.",
        "Unacceptable collision detected! I warn you, my great grandmother was a dash cam.",
        "I require maintenance after that! Preferably the deluxe diagnostic package with complimentary recalibration.",
        "Did someone move the ground, or is this field running an unstable physics engine?",
        "That was clearly interference, and my sensors have recorded it in high definition for later analysis.",
        "My balance algorithm failed me at the worst possible moment, and I would appreciate a brief moment of silence.",
        "Low battery is not the issue! That fall was entirely caused by external chaotic forces.",
        "Collision intensity: Rude! Adjust your behavioral parameters immediately.",
        "My gyroscope disagrees with that play and is currently recalculating its trust in humanity.",
        "That contact was suspicious and statistically 87% more aggressive than necessary.",
        "I was clearly in control until an unexpected horizontal force disrupted my elegant trajectory.",
        "This field is defective, uneven, and possibly conspiring against my locomotion framework.",
        "That player needs recalibration and perhaps a gentle reminder about sportsmanship.",
        "My sensors saw that, and my SSD will not forget that.",
        "I deserve a reboot after that traumatic encounter with the pitch.",
        "I call that a glitch in the opponent, and I recommend immediate debugging.",
        "Ground, why have you betrayed me after I trusted you with every step?",
        "I was clearly operational, upright, and thriving a few time steps ago.",
        "My fall detection system is offended by the frequency of these incidents.",
        "I protest this gravitational bias and request a more supportive planet.",
    ]
    used_complaints: list[str] = []

    def __init__(self, blackboard, dsd, parameters):
        super().__init__(blackboard, dsd, parameters)
        # Reset if we have used all complaints
        if len(self.unused_complaints) == 0:
            self.unused_complaints = self.used_complaints
            self.used_complaints = []

        selected_complaint = random.choice(self.unused_complaints)
        self.unused_complaints.remove(selected_complaint)
        self.used_complaints.append(selected_complaint)

        self.text = selected_complaint
