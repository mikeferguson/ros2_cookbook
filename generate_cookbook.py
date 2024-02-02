#!/usr/bin/env python
import pathlib

import mkdocs_gen_files
import tomli


# Parameters
config_file = pathlib.Path(__file__).parent/"cookbook_config.toml"
config = tomli.loads(config_file.read_text())
client_libraries = config["client_libraries"]
nav_file = "literate-nav.md"

# Define some functions
def get_client_lib_md_label(client_lib: dict) -> str:
    return f"{client_lib['icon']:s} {client_lib['name']:s} ({client_lib['path']:s})"

def get_client_lib_md_link(client_lib: dict) -> str:
    return f"[{get_client_lib_md_label(client_lib):s}]({client_lib['path']:s}/index.md)"

# Navigation creation for the client libraries
literate_nav = mkdocs_gen_files.Nav()

# Generate the client libraries index page
client_libs_path = pathlib.Path("client_libraries")
client_libs_index_path = client_libs_path/"index.md"
client_libs_index_text = "\n".join([
    "<div class=\"centered-content\" markdown>",
    "# Client libraries\n",
    *[f"##{get_client_lib_md_link(cl)}" for cl in client_libraries],
    "\n</div>",
])
with mkdocs_gen_files.open(client_libs_index_path, "w") as fd:
    print(client_libs_index_text, file=fd)

literate_nav["Client Libraries"] = client_libs_index_path.as_posix()

# Create the client libraries pages
for client_lib in client_libraries:
    for path in sorted(pathlib.Path(client_lib["path"]).rglob("*.md")):
        client_lib_page_path = client_libs_path/path
        with mkdocs_gen_files.open(client_lib_page_path, "w") as fd:
            print(path.read_text(), file=fd)

        mkdocs_gen_files.set_edit_path(client_lib_page_path, path)

        # Get the path's parts
        nav_path = ["Client Libraries", client_lib["name"]]
        if (page_name := path.with_suffix("").name) != "index":
            nav_path.append(page_name.capitalize())

        literate_nav[nav_path] = client_lib_page_path.as_posix()

# Generate the other index page
other_path = pathlib.Path("other")

for path in sorted(pathlib.Path("pages").rglob("*.md")):
    page_path = other_path/path
    with mkdocs_gen_files.open(page_path, "w") as fd:
        print(path.read_text(), file=fd)

    mkdocs_gen_files.set_edit_path(page_path, path)

    # Get the path's parts
    nav_path = ["Other"]
    if (page_name := path.with_suffix("").name) != "index":
        nav_path.append(page_name.capitalize())

    literate_nav[nav_path] = page_path.as_posix()

with mkdocs_gen_files.open(nav_file, "w") as nf:
    nf.writelines(literate_nav.build_literate_nav())
