# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "Codestral ROS2 Generator"
copyright = "2025, Lexmaister"
author = "Lexmaister"
version = "0.2.0"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

import sys
from codestral_ros2_gen import get_project_root

sys.path.insert(0, get_project_root())  # Add project root to path

# Extensions
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
    "sphinx.ext.napoleon",
    "sphinx.ext.intersphinx",
    "sphinx_autodoc_typehints",
    "myst_parser",
]

templates_path = ["_templates"]
exclude_patterns = []

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output


# Theme configuration
html_theme = "alabaster"  # default
html_theme = "sphinx_rtd_theme"
# Custom output directory
html_output_path = "html"
# html_static_path = ["_static"]

# Napoleon settings for docstring parsing
napoleon_google_docstring = True
napoleon_numpy_docstring = False
napoleon_include_init_with_doc = True
napoleon_include_private_with_doc = True

# autodoc configuration
autodoc_member_order = "bysource"
autodoc_typehints = "description"

# suppress warnings for ``` markers
suppress_warnings = [
    "docutils.parsers.rst.inline_literal",
]

import warnings


def setup(app):
    warnings.filterwarnings(
        "ignore",
        message="Inline literal start-string without end-string",
        module="docutils.parsers.rst.states",
    )
