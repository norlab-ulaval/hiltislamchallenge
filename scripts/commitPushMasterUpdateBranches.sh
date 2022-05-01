#! /bin/bash
git fetch

git checkout master
git commit -m "$1"
git push

git checkout construction
git merge master --commit
git push

git checkout corridor
git merge master --commit
git push

git checkout outdoor
git merge master --commit
git push

git checkout theater
git merge master --commit
git push

git checkout master