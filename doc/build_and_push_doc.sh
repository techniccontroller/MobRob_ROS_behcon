make html BUILDDIR=/tmp/mobrob_behcon_doc_build
cd /tmp/mobrob_behcon_doc_build
git clone https://github.com/techniccontroller/MobRob_ROS_behcon.git
cd ./MobRob_ROS_behcon
git stash
git checkout --force gh-pages
git clean -fd
rm -vrf *
rm -rf /tmp/mobrob_behcon_doc_build/html/.doctrees
cp -vr /tmp/mobrob_behcon_doc_build/html/. .
touch ".nojekyll"
git add .
git commit --allow-empty -m "Add changes for $GITHUB_SHA"
git push
