package frc.robot;

public enum SparkLedPattern {

    // ----- Color 1 Patterns -----
    COLOR1_BREATH_FAST(0.11),
    COLOR1_SHOT(0.13),
    COLOR1_STROBE(0.15),

    // ----- Color 2 Patterns -----
    COLOR2_END_TO_END_BLEND_TO_BLACK(0.17),
    COLOR2_LARSON_SCANNER(0.19),
    COLOR2_LIGHT_CHASE(0.21),
    COLOR2_HEARTBEAT_SLOW(0.23),
    COLOR2_HEARTBEAT_MEDIUM(0.25),
    COLOR2_HEARTBEAT_FAST(0.27),
    COLOR2_BREATH_SLOW(0.29),
    COLOR2_BREATH_FAST(0.31),
    COLOR2_SHOT(0.33),
    COLOR2_STROBE(0.35),

    // ----- Color 1 & 2 Patterns -----
    SPARKLE_COLOR1_ON_COLOR2(0.37),
    SPARKLE_COLOR2_ON_COLOR1(0.39),
    COLOR_GRADIENT_1_AND_2(0.41),
    BEATS_PER_MINUTE_1_AND_2(0.43),
    END_TO_END_BLEND_1_TO_2(0.45),
    END_TO_END_BLEND(0.47),
    COLOR1_AND_COLOR2_NO_BLENDING(0.49),
    TWINKLES_1_AND_2(0.51),
    COLOR_WAVES_1_AND_2(0.53),
    SINELON_1_AND_2(0.55),

    // ----- Solid Colors -----
    HOT_PINK(0.57),
    DARK_RED(0.59),
    RED(0.61),
    RED_ORANGE(0.63),
    ORANGE(0.65),
    GOLD(0.67),
    YELLOW(0.69),
    LAWN_GREEN(0.71),
    LIME(0.73),
    DARK_GREEN(0.75),
    GREEN(0.77),
    BLUE_GREEN(0.79),
    AQUA(0.81),
    SKY_BLUE(0.83),
    DARK_BLUE(0.85),
    BLUE(0.87),
    BLUE_VIOLET(0.89),
    VIOLET(0.91),
    WHITE(0.93),
    GRAY(0.95),
    DARK_GRAY(0.97),
    BLACK(0.99);

    private final double value;

    SparkLedPattern(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }
}
