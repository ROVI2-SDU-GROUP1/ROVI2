#!/bin/bash

set -o nounset
set -o errexit

# Where to place the generated documentation and associated files
documentation_output_dir="$(pwd)/generated"
[ -d "${documentation_output_dir}" ] || mkdir "${documentation_output_dir}"
documentation_utils_output_dir="$(pwd)/generated-utils"
[ -d "${documentation_utils_output_dir}" ] || mkdir "${documentation_utils_output_dir}"
documentation_output_assets_dir="$(pwd)/generated/assets"
[ -d "${documentation_output_assets_dir}" ] || mkdir "${documentation_output_assets_dir}"


yaml_collection_of_tagfiles="${documentation_utils_output_dir}/collection_of_tagfiles.yaml"
yaml_collection_of_tagfiles_excluding_current_package="${documentation_utils_output_dir}/collection_of_tagfiles_excluding_current_package.yaml"
# Clean out the collection of tagfiles
[ -f "${yaml_collection_of_tagfiles}" ] && rm "${yaml_collection_of_tagfiles}"
[ -f "${yaml_collection_of_tagfiles_excluding_current_package}" ] && rm "${yaml_collection_of_tagfiles_excluding_current_package}"

#$1=package_path
function generate_assets () {
    # Generate GraphViz dot files (hardcoded to be layed out with dot)
    # Could also use 'rosls ${1}/assets' - but then the directory can't easily be verified to exist
    pkg_path="$(rospack find ${1})"
    if [ -d "${pkg_path}/assets" ]; then
	for f in $(cd ${pkg_path}/assets/; ls *.gv); do
	    echo "asset: ${f}"
	    asset_name="$(echo ${f} | sed -e 's/\.gv$//')"
	    dot -Tpng ${pkg_path}/assets/${f} -o${documentation_output_assets_dir}/${asset_name}.png
	done
	# Simply just copy png files (and other image files that we may be adding)
	for f in $(cd ${pkg_path}/assets/; ls *.png); do
	    echo "asset: ${f}"
	    if [ -f "${documentation_output_assets_dir}/${f}" ]; then
		echo "asset '${documentation_output_assets_dir}/${f}' already exists - overwriting!"
	    fi
	    cp "${pkg_path}/assets/${f}" "${documentation_output_assets_dir}/${f}"
	done
    else
	echo "${pkg_path}/assets doesn't exist - skipping creating assets"
    fi
}

#$1=package_path
function extract_package_name_from_package_path () {
    trimmed_package_path=$(echo "${1}" | sed -e 's/\/package.xml//g')
    package_name=$(echo ${trimmed_package_path} | sed -e 's/.*\///g')
    echo "${package_name}"
}

#$1=package_name, $2=tagfile_path
function add_tagfile_to_yaml_collection_of_tagfiles () {
    # The relative walking up through parent directories should be ../../../ (c++, html, and project name) - but changed to only walk up two times, as the documentation is being generated in a subfolder specified via the -o options to rosdoc_lite. And due to the change introduced in https://github.com/ros-infrastructure/rosdoc_lite/commit/34b448d90e4fe453570da5172a544f70190b9499 then the generated cross-referencing links would go up four times instead of the required and working three times - so this change is a workaround.
    cat >> "${yaml_collection_of_tagfiles}" <<EOF
- docs_url: ../../${1}/html/c++
  location: file://${2}
EOF
}

#$1=package_name, $2=use_tagfiles{yes|no}
function generate_documentation_and_tagfile () {
    absolute_path_to_package="$(rospack find ${1})"
    package_documentation_output_dir="${documentation_output_dir}/${1}"
    package_tagfile="${documentation_utils_output_dir}/${1}.tag"
    [ -d "${package_documentation_output_dir}" ] && rm -r "${package_documentation_output_dir}"
    if [ "${2}" = "yes" ]; then
        rosdoc_lite -o "${package_documentation_output_dir}" --tagfile="${yaml_collection_of_tagfiles_excluding_current_package}" --generate_tagfile="${package_tagfile}" "${absolute_path_to_package}"
    else
        rosdoc_lite -o "${package_documentation_output_dir}" --generate_tagfile="${package_tagfile}" "${absolute_path_to_package}"
        add_tagfile_to_yaml_collection_of_tagfiles "${1}" "${package_tagfile}"
    fi
}

#$1=package_name
function generate_documentation_using_tagfiles () {
    sed -e "/\/${1}\(\/\|\.\)/d" "${yaml_collection_of_tagfiles}" > "${yaml_collection_of_tagfiles_excluding_current_package}"
    generate_documentation_and_tagfile "${1}" "yes"
    # Cleanup the temporary file
    [ -f "${yaml_collection_of_tagfiles_excluding_current_package}" ] && rm "${yaml_collection_of_tagfiles_excluding_current_package}"
}

########################################################################
#### Main
########################################################################
# Find all CAROS packages
found_packages="$(find ../ -name 'package.xml')"
package_names=""

# Get package names
for found_package in ${found_packages}; do
    package_names="${package_names} $(extract_package_name_from_package_path ${found_package})"
done

# Generate documentation assets
for package_name in ${package_names}; do
    echo "------------------------------------------------------------------------"
    echo "Generating documentation assets for ${package_name}"
    echo "------------------------------------------------------------------------"
    generate_assets "${package_name}" > "${documentation_utils_output_dir}/${package_name}_assets.log" 2>&1
done

# Generate documentation and tagfiles
for package_name in ${package_names}; do
    echo "------------------------------------------------------------------------"
    echo "Generating documentation and tagfile for ${package_name}"
    echo "------------------------------------------------------------------------"
    generate_documentation_and_tagfile "${package_name}" "no" > "${documentation_utils_output_dir}/${package_name}.log" 2>&1
done

# Generate documentation using tagfiles
for package_name in ${package_names}; do
    echo "########################################################################"
    echo "Generating documentation using tagfiles for ${package_name}"
    echo "########################################################################"
    generate_documentation_using_tagfiles "${package_name}" > "${documentation_utils_output_dir}/${package_name}_with_tagfiles.log" 2>&1
done

########################################################################
#### Index
########################################################################
echo "########################################################################"
echo "Generating index"
echo "########################################################################"
index_file="${documentation_output_dir}/index.html"
[ -f "${index_file}" ] && rm "${index_file}"

cat >> "${index_file}" <<EOF
<html>
<head>
<title>CAROS Documentation Index</title>
</head>
<body>
EOF

# Either run throug the package names, or through the directories found in ${documentation_output_dir}
for package_name in ${package_names}; do
    cat >> "${index_file}" <<EOF
<a href="${package_name}/html/index-msg.html">${package_name}</a><br />
EOF
done

cat >> "${index_file}" <<EOF
</body>
</html>
EOF


exit 0

# sort and unique, to remove old entries in the tagfiles.yaml or simply remove it (make a backup) before each run/invocation of this script or similar
