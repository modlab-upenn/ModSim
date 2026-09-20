"""Assembly color allocation avoids ID-hash collisions as topology changes."""

from modsim_studio.visual_colors import CategoryColors


def test_assembly_colors_survive_reordering_and_new_members_without_collisions() -> None:
    colors = CategoryColors()
    # These two identifiers collided in the previous weighted-hash palette.
    identifiers = ["assembly:module_1", "assembly:module_9"]
    colors.prepare(identifiers)
    original = {identifier: colors.color(identifier, "light") for identifier in identifiers}
    assert len(set(original.values())) == 2

    colors.prepare(["assembly:new", *reversed(identifiers)])
    assert all(colors.color(i, "light") == original[i] for i in identifiers)
    assert colors.color("assembly:new", "light") not in original.values()


def test_assembly_colors_recover_distinct_hues_after_population_shrinks() -> None:
    colors = CategoryColors()
    identifiers = [f"assembly:{i:02d}" for i in range(20)]
    colors.prepare(identifiers)
    # A large population must cycle the palette; shrinking must free those hues.
    remaining = identifiers[::3]
    for visible in (remaining, list(reversed(identifiers[:9])), remaining):
        colors.prepare(visible)
        for theme in ("graphite", "midnight", "light"):
            assert len({colors.color(i, theme) for i in visible}) == len(visible)
