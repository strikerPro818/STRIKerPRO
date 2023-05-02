from ultralytics.vit import SAM
import cv2

# from ultralytics.vit import MODEL_TYPe

model = SAM("sam_b.pt")
results = model.predict(source=2,stream=True,show=False)
for result in results:
    print(result.boxes)
    # img = results.orig_img
    # print(img)
    # cv2.imgshow(img)
# display model information
# for result in results:
#     print(result.speed)
# model.predict(...)  # train the model
