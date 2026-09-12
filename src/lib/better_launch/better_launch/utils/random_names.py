import random


class UniqueWordGenerator:
    def __init__(self):
        # English roots and stems
        self.roots = [
            'amber', 'arch', 'ash', 'bran', 'brass', 'bronze', 'cedar', 'crim', 'crystal',
            'dawn', 'dusk', 'ember', 'falcon', 'frost', 'gild', 'glim', 'gold', 'haven',
            'hawk', 'iron', 'mist', 'moon', 'moss', 'oak', 'pearl', 'quick',
            'raven', 'rock', 'rose', 'scar', 'shadow', 'silver', 'star', 'stone', 'storm',
            'thorn', 'thunder', 'whis', 'wind', 'winter', 'wolf', 'wood',
            'bar', 'bor', 'car', 'cor', 'dar', 'dor', 'far', 'for', 'gar', 'gor',
            'har', 'hor', 'jar', 'jor', 'kar', 'kor', 'lar', 'lor', 'mar', 'mor',
            'nar', 'nor', 'par', 'por', 'sar', 'sor', 'tar', 'tor', 'var', 'vor',
            'wal', 'wel', 'wil', 'zar', 'zel', 'bel', 'cal', 'del', 'fel', 'gel',
            'kel', 'mel', 'nel', 'pel', 'rel', 'sel', 'tel', 'vel', 'cran', 'gran'
        ]
        
        # English suffixes and endings
        self.suffixes = [
            'ton', 'ston', 'den', 'don', 'ley', 'ly', 'ney', 'win', 'wyn', 'kin',
            'lin', 'ling', 'son', 'sen', 'man', 'mon', 'rick', 'wick', 'ford',
            'worth', 'borne', 'burn', 'by', 'dale', 'field', 'gate', 'ham', 'hold',
            'ard', 'art', 'ent', 'ant', 'er', 'or', 'ar', 'ian', 'ean', 'an',
            'ous', 'ious', 'eous', 'ace', 'ice', 'ance', 'ence', 'age', 'ine',
            'ing', 'ed', 'en', 'eth', 'ith', 'us', 'is', 'as', 'os'
        ]
        
        # Pregenerate all possible combinations as indices
        self.available_indices = list(range(len(self.roots) * len(self.suffixes)))
        random.shuffle(self.available_indices)
        self.next_index = 0
    
    def get_unique_word(self) -> str:
        """Generate the next unique word"""
        if self.next_index >= len(self.available_indices):
            raise ValueError("No more unique fantasy words available")
        
        while True:
            # Get the next index and convert to root/suffix pair
            idx = self.available_indices[self.next_index]
            self.next_index += 1
            
            root_idx = idx // len(self.suffixes)
            suffix_idx = idx % len(self.suffixes)
            
            # Avoid some awkward combinations
            word = self.roots[root_idx] + self.suffixes[suffix_idx]
            if not any(x in word for x in ("shsh", "thth")):
                break
        
        return word
    
    def reset(self):
        """Shuffle and reset the generator"""
        random.shuffle(self.available_indices)
        self.next_index = 0
    
    def remaining(self) -> int:
        """Return number of words still available"""
        return len(self.available_indices) - self.next_index
    
    def total_available(self) -> int:
        """Return total number of unique words possible"""
        return len(self.available_indices)


default_name_generator = UniqueWordGenerator()


def get_unique_word() -> str:
    return default_name_generator.get_unique_word()


if __name__ == "__main__":
    discovered = set()
    samples = 10000
    collisions = 0

    for i in range(default_name_generator.remaining()):    
        n = default_name_generator.get_unique_word()
        
        if i % 100 == 0:
            print(n)

        if n in discovered:
            collisions += 1
        else:
            discovered.add(n)

    print(f"Collisions: {collisions}/{samples}")
