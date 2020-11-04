git clone https://github.com/vishalgpt579/sahayak_bot.git temp
cd temp
rm -rf .git

cd ebot_description
echo "Copying meshes"
mv meshes ../../sahayak_bot/ebot_description

cd ../ebot_gazebo
echo "Copying models"
mv models ../../sahayak_bot/ebot_gazebo

rm -rf ../../temp

