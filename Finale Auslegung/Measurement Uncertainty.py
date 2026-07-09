from pyfluids import FluidsList

print([x for x in dir(FluidsList) if "Prop" in x or "prop" in x or "290" in x or "134" in x])