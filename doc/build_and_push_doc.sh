tmp_dir=$(mktemp -d -t pages-XXXXXXXXXX)
make html BUILDDIR=$tmp_dir
cd $tmp_dir
git clone https://github.com/techniccontroller/MobRob_ROS_behcon.git
cd ./MobRob_ROS_behcon
git stash
git checkout --force gh-pages
git clean -fd
rm -vrf *
rm -rf $tmp_dir/html/.doctrees
cp -vr $tmp_dir/html/. .
touch ".nojekyll"
git add .
git commit --allow-empty -m "Add changes for $GITHUB_SHA"
git push
