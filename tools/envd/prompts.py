"""Class taxonomy and zero-shot prompt ensembles of envd (fixed before scoring, 2026-10-26 17:2x CEST).

Three independent groups; the daemon publishes a probability over each group separately.
Prompt ensemble: every template x every class phrase; the class text embedding is the
L2-normalised mean of the normalised prompt embeddings (the usual CLIP ensemble).
"""

GROUPS = {
    "place": ["highway", "city_street", "residential", "rural_road", "forest_road", "tunnel",
              "parking_lot", "underground_parking", "gas_station", "bridge"],
    "weather": ["clear", "overcast", "rain", "snow", "fog"],
    "light": ["day", "dusk_dawn", "night"],
}

TEMPLATES = {
    "place": ["a photo taken from a car driving on {}.", "a dashcam photo of {}.",
              "the view through the windshield of a car on {}.", "a forward-facing camera image of {}."],
    "weather": ["a dashcam photo of a road in {} weather.", "a photo taken from a car, {}.",
                "a view through the windshield of a car, {}.", "driving in {}."],
    "light": ["a dashcam photo of a road {}.", "a photo taken from a car {}.",
              "the view through the windshield of a car {}.", "driving {}."],
}

# Several phrasings per class; each goes through every template of its group.
PHRASES = {
    # place: fills "driving on {}"
    "highway": ["a highway", "a motorway with several lanes", "a freeway", "an expressway"],
    "city_street": ["a city street", "a busy street in a city centre with shops", "a high street with shops and pedestrians",
                    "a downtown street with traffic lights"],
    "residential": ["a quiet residential street", "a street in a residential neighbourhood with houses",
                    "a suburban street with houses and parked cars"],
    "rural_road": ["a rural road through fields", "a country road through open countryside", "a road through farmland",
                   "a wild road with no buildings"],
    "forest_road": ["a road through a forest", "a forest road with trees on both sides", "a road through woods"],
    "tunnel": ["a road inside a tunnel", "a road tunnel with artificial lights on the ceiling",
               "a tunnel with concrete walls"],
    "parking_lot": ["a parking lot", "an outdoor car park with parked cars", "a parking area in front of a building"],
    "underground_parking": ["an underground parking garage", "an indoor parking garage with concrete pillars",
                            "a multi-storey car park, indoors"],
    "gas_station": ["a gas station forecourt", "a petrol station with fuel pumps and a canopy"],
    "bridge": ["a bridge over water", "a long bridge with railings", "a viaduct"],
    # weather: fills "in {} weather" / ", {}"
    "clear": ["clear", "sunny", "clear sky"],
    "overcast": ["overcast", "cloudy", "grey overcast sky"],
    "rain": ["rainy", "rain with wet road and raindrops on the windshield", "heavy rain"],
    "snow": ["snowy", "snow on the road and roadside", "snowfall"],
    "fog": ["foggy", "dense fog with low visibility", "mist"],
    # light: fills "a road {}"
    "day": ["in daylight", "during the day", "in bright daylight"],
    "dusk_dawn": ["at dusk", "at dawn", "at sunset with a dim sky", "in twilight"],
    "night": ["at night", "in the dark at night with headlights and street lights", "late at night"],
}


def prompts_for(group, cls):
    return [t.format(p) for t in TEMPLATES[group] for p in PHRASES[cls]]
