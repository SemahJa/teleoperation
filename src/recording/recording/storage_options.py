"""MCAP chunk compression settings; payloads and timestamps remain lossless."""
ALGORITHMS = ("None", "Zstd")
LEVELS = ("Fastest", "Fast", "Default", "Slow", "Slowest")


def mcap_storage_options(compression="None", level="Fast"):
    if compression not in ALGORITHMS:
        raise ValueError("mcap_compression must be None/Zstd")
    if level not in LEVELS:
        raise ValueError("invalid mcap_compression_level")
    result = dict(compression=compression, noChunkCRC=False, noMessageIndex=False)
    if compression != "None":
        result["compressionLevel"] = level
    return result
