
from ultralytics import YOLO
import cv2
import numpy as np
from PIL import Image
import time
import os
from shutil import copyfile
from torchvision import datasets
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
from collections import Counter
import argparse
import scipy.io
import torch
import threading
import logging
import argparse
import torch.nn as nn
import torch.optim as optim
from torch.optim import lr_scheduler
from torch.autograd import Variable
import torch.backends.cudnn as cudnn
import numpy as np
from torchvision import datasets, models, transforms
import time
import os
import scipy.io
import yaml
import math
from tqdm import tqdm
from model import ft_net, ft_net_dense, ft_net_hr, ft_net_swin, ft_net_swinv2, ft_net_efficient, ft_net_NAS, ft_net_convnext, PCB, PCB_test
from utils import fuse_all_conv_bn
version =  torch.__version__


# Load the YOLO model
## 갤러리에 저장
class Detector:
    def __init__(self,event):
        self.event= event
        self.model = YOLO("yolov8n.pt", verbose=False) #객체 검출에 사용될 모델  #수정
       #수정
        self.data_dir='/home/hyeonuk/python/Person_reID_baseline_pytorch/Market/pytorch'                # 데이터 셋 경로 
        self.gallery_path = '/home/hyeonuk/python/Person_reID_baseline_pytorch/Market/pytorch/gallery'  # 갤러리 경로
        self.query_path='/home/hyeonuk/python/Person_reID_baseline_pytorch/Market/pytorch/query'        # 쿼리 경로
        
        self.image_datasets = {x: datasets.ImageFolder( os.path.join(self.data_dir,x) ) for x in ['gallery','query']}   # 쿼리랑 갤러리 이미지 데이터 셋 

        # Get a list of all subdirectories in the gallery path
        self.existing_folders = [d for d in os.listdir(self.gallery_path) if os.path.isdir(os.path.join(self.gallery_path, d))]
        self.existing_ids = []

        self.target_width = 226 # 이미지 Resize
        self.target_height = 476

    
    def save_gallery2(self,flag,id):
        for folder in self.existing_folders:
            try:
                folder_id = int(folder)
                self.existing_ids.append(folder_id)
            except ValueError:
                # Skip folders that do not have numeric names
                continue

        # Determine the next ID
        if self.existing_ids:
            next_id = max(self.existing_ids) + 1
        else:
            next_id = 1
            
        # 갤러리 생성 여부에 따른 id 고정
        if flag==True:
            self.new_folder_name=str(next_id)
        else:
            self.new_folder_name = str(max(self.existing_ids))

        self.id=id
        # Create the full path for the new directory
        self.gallery_folder_path = os.path.join(self.gallery_path, self.new_folder_name)
        self.query_folder_path=os.path.join(self.query_path,self.new_folder_name)
        self.target_query_folder_path=os.path.join(self.query_path,str(self.id)) # 이 쿼리 폴더만 업데이트함
        
        # Check if the directory already exists
        if not os.path.isdir(self.gallery_folder_path):
            # 갤러리 폴더 생성
            os.mkdir(self.gallery_folder_path)
            print(f"gallery {self.new_folder_name} created successfully.")
        else:
            print(f"gallery {self.new_folder_name} already exists.")

            
        if not os.path.isdir(self.query_folder_path):
            # 쿼리 폴더 생성
            os.mkdir(self.query_folder_path)
            print(f"query {self.new_folder_name} created successfully.")
        else:
            print(f"query {self.new_folder_name} already exists.")
            
            
        if flag==True:
            cap = cv2.VideoCapture(2)
            count=1
            max_gal_count=250
            while True:
                # Read frame from video capture
                ret, frame = cap.read()
                if not ret:
                    break
                # Perform object detection and tracking
                results = self.model(frame, conf=0.8, iou=0.8)
                
                # 갤러리에 데이터 저장
                for detection in results[0].boxes:
                    if detection.cls[0] == 0:  # Person class
                        count +=1

                        bbox = detection.xyxy[0].cpu().numpy().astype(int)
                        x1, y1, x2, y2 = bbox
                        
                        # Extract the bounding box from the frame
                        person_img = frame[y1:y2, x1:x2]
                        resized_img = cv2.resize(person_img, (self.target_width, self.target_height)) # Resize
                        person_pil = Image.fromarray(cv2.cvtColor(resized_img, cv2.COLOR_BGR2RGB))     
                        
                        # 갤러리 저장
                        if count % 10==0 and count <= max_gal_count:
                            gallery_image_index = count//10
                            gallery_image_path = os.path.join(self.gallery_folder_path, f"{self.new_folder_name}_c{gallery_image_index}.jpg")
                            if gallery_image_index==1:
                                query_image_path = os.path.join(self.query_folder_path, f"{self.new_folder_name}_c{gallery_image_index}.jpg")
                                person_pil.save(query_image_path)
                            person_pil.save(gallery_image_path) #갤러리 경로에 저장
                            print("gallery_img_save",gallery_image_index)
                            
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
                        
                cv2.imshow("YOLOv8 Tracking", frame)
            
                # Exit if 'q' is pressed
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                if count >max_gal_count:
                    break

            # Release video capture and destroy windows
            cap.release()
            cv2.destroyAllWindows()

   
# Initialize video capture (0 for webcam or provide video file path)
    def detect_query(self):
        #time.sleep(5)
        cap = cv2.VideoCapture(2)
        count=1
        query_count=1
        
        while True:
            # Read frame from video capture
            ret, frame = cap.read()
            if not ret:
                break
            
            # Perform object detection and tracking
            results = self.model(frame, conf=0.8, iou=0.8,)
            logging.basicConfig(level=logging.WARNING)
            logging.getLogger().setLevel(logging.ERROR)
            
            # 갤러리에 데이터 저장
            for detection in results[0].boxes:
                if detection.cls[0] == 0:  # Person class
                    count +=1
                    query_count +=1

                    bbox = detection.xyxy[0].cpu().numpy().astype(int)
                    x1, y1, x2, y2 = bbox                                     # 바운딩 박스의 좌표
                    
                    # Extract the bounding box from the frame
                    person_img = frame[y1:y2, x1:x2]                          
                    resized_img = cv2.resize(person_img, (self.target_width, self.target_height))
                    person_pil = Image.fromarray(cv2.cvtColor(resized_img, cv2.COLOR_BGR2RGB))
        
                    # 쿼리 저장
                    if count > 1:       
                        # 쿼리 저장 주기
                        if query_count % 2==0:
                            query_image_path=os.path.join(self.target_query_folder_path,f"{str(self.id)}_c{1}.jpg") # 지정한 id 쿼리 폴더
                            person_pil.save(query_image_path) # 저장
                            #person_pil.save(multi_query_image_path) 
                            
                        if query_count>100:
                            query_count=1
                    
                # Visualize detection
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 1)
                #Show the frame with detections
                cv2.imshow("YOLOv8 Tracking", frame)
        
            # Exit if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


        # Release video capture and destroy windows
        cap.release()
        cv2.destroyAllWindows()
        

        
        
#############################################################################################

class ReIDTester:
    def __init__(self,event):
        self.opt = self.parse_args()
        self.load_config()
        self.set_gpu()
        self.get_data_transforms()
        self.load_data()
        self.load_model()
        self.event=event
        self.gallery_path='/home/hyeonuk/python/Person_reID_baseline_pytorch/Market/pytorch/gallery'  
        self.existing_folders = [d for d in os.listdir(self.gallery_path) if os.path.isdir(os.path.join(self.gallery_path, d))]
        self.existing_ids = []
        self.check_id()
        
    def check_id(self):
        for folder in self.existing_folders:
            try:
                folder_id = int(folder)
                self.existing_ids.append(folder_id)
            except ValueError:
                # Skip folders that do not have numeric names
                continue

    
    def sort_img(self,qf, ql, qc, gf, gl, gc):
        query = qf.view(-1, 1)
        score = torch.mm(gf, query)
        score = score.squeeze(1).cpu()
        score = score.numpy()
        index = np.argsort(score)[::-1]

        query_index = np.argwhere(gl == ql)
        camera_index = np.argwhere(gc == qc)
        junk_index1 = np.argwhere(gl == -1)
        junk_index2 = np.intersect1d(query_index, camera_index)
        junk_index = np.append(junk_index2, junk_index1)

        mask = np.in1d(index, junk_index, invert=True)
        index = index[mask]
        return index
    # 수정 x
    def imshow(self,path, title=None):
        im = plt.imread(path)
        plt.imshow(im)
        if title is not None:
            plt.title(title)
        plt.pause(0.001)  # pause a bit so that plots are updated
        
    # 확인
    def parse_args(self):# 수정
        parser = argparse.ArgumentParser(description='Test')
        parser.add_argument('--gpu_ids', default='0', type=str, help='gpu_ids: e.g. 0  0,1,2  0,2') # gpu 0번
        parser.add_argument('--which_epoch', default='last', type=str, help='0,1,2,3...or last')    # last 학습 데이터 사용
        parser.add_argument('--test_dir', default='/home/hyeonuk/python/Person_reID_baseline_pytorch/Market/pytorch', type=str, help='./test_data')   # 테스트_dir 경로 수정
        parser.add_argument('--name', default='ft_ResNet50', type=str, help='save model path')          #model에 있는 ft_ResNet50 사용
        parser.add_argument('--batchsize', default=256, type=int, help='batchsize')                     
        parser.add_argument('--linear_num', default=512, type=int, help='feature dimension: 512 or default or 0 (linear=False)')
        parser.add_argument('--use_dense', action='store_true', help='use densenet121')
        parser.add_argument('--use_efficient', action='store_true', help='use efficient-b4')
        parser.add_argument('--use_hr', action='store_true', help='use hr18 net')
        parser.add_argument('--PCB', action='store_true', help='use PCB')
        parser.add_argument('--multi', action='store_true', help='use multiple query')
        parser.add_argument('--fp16', action='store_true', help='use fp16.')
        parser.add_argument('--ibn', action='store_true', help='use ibn.')
        parser.add_argument('--ms', default='1', type=str, help='multiple_scale: e.g. 1 1,1.1  1,1.1,1.2')
        return parser.parse_args()
    # 확인
    def load_config(self):
        self.config_path = os.path.join('./model', self.opt.name, 'opts.yaml')      #  model.ft_ResNet50에 있는 yaml 파일  수정  model 파일이 현재 디렉토리에 있어야 함
        with open(self.config_path, 'r') as stream:
            self.config = yaml.load(stream, Loader=yaml.FullLoader)
        self.opt.fp16 = self.config['fp16']
        self.opt.PCB = self.config['PCB']
        self.opt.use_dense = self.config['use_dense']
        self.opt.use_NAS = self.config.get('use_NAS', False)
        self.opt.stride = self.config['stride']
        self.opt.use_swin = self.config.get('use_swin', False)
        self.opt.use_swinv2 = self.config.get('use_swinv2', False)
        self.opt.use_convnext = self.config.get('use_convnext', False)
        self.opt.use_efficient = self.config.get('use_efficient', False)
        self.opt.use_hr = self.config.get('use_hr', False)
        self.opt.nclasses = self.config.get('nclasses', 10)
        self.opt.ibn = self.config.get('ibn', False)
        self.opt.linear_num = self.config.get('linear_num', self.opt.linear_num)

    #확인
    def set_gpu(self):
        self.str_ids = self.opt.gpu_ids.split(',')
        self.gpu_ids = [int(str_id) for str_id in self.str_ids if int(str_id) >= 0]
        if len(self.gpu_ids) > 0:
            torch.cuda.set_device(self.gpu_ids[0])
            cudnn.benchmark = True
        
        self.str_ms=self.opt.ms.split(',')
        self.ms=[]
        for s in self.str_ms:
            self.s_f = float(s)
            self.ms.append(math.sqrt(self.s_f))
        
    #확인
    def get_data_transforms(self):
        if self.opt.use_swin:
            self.h, self.w = 224, 224
        else:
            self.h, self.w = 256, 128

        self.data_transforms = transforms.Compose([
            transforms.Resize((self.h, self.w), interpolation=3),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])

        if self.opt.PCB:
            self.data_transforms = transforms.Compose([
                transforms.Resize((384, 192), interpolation=3),
                transforms.ToTensor(),
                transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
            ])
            self.h, self.w = 384, 192
    #확인
    def load_data(self):
        print('new_load_data success\n')
        #parser.add_argument('--test_dir', default='/home/hyeonuk/python/Person_reID_baseline_pytorch/Market/pytorch', type=str, help='./test_data')  
        self.data_dir = self.opt.test_dir

        if self.opt.multi:
            self.image_datasets = {x: datasets.ImageFolder(os.path.join(self.data_dir, x), self.data_transforms) for x in
                              ['gallery', 'query', 'multi-query']} # 갤러리, 쿼리의 이미지 데이터셋 
            self.dataloaders = {x: torch.utils.data.DataLoader(self.image_datasets[x], batch_size=self.opt.batchsize,
                                                          shuffle=False, num_workers=0) for x in
                           ['gallery', 'query', 'multi-query']}
        else:
            self.image_datasets = {x: datasets.ImageFolder(os.path.join(self.data_dir, x), self.data_transforms) for x in
                              ['gallery', 'query']}
            self.dataloaders = {x: torch.utils.data.DataLoader(self.image_datasets[x], batch_size=self.opt.batchsize,
                                                          shuffle=False, num_workers=0) for x in ['gallery', 'query']} #default 16
        self.class_names = self.image_datasets['query'].classes

    #확인
    def load_network(self, network):                # model.Res_net에 있는 last.pth 사용
        self.save_path = os.path.join('./model', self.opt.name, f'net_{self.opt.which_epoch}.pth')
        try:
            network.load_state_dict(torch.load(self.save_path))
        except:
            version = torch.__version__
            if torch.cuda.get_device_capability()[0] > 6 and len(self.opt.gpu_ids) == 1 and int(version[0]) > 1:
                torch.set_float32_matmul_precision('high')
                network = torch.compile(network, mode="default", dynamic=True)
            network.load_state_dict(torch.load(self.save_path))
        return network
    #확인
    def fliplr(self, img):
        '''flip horizontal'''
        inv_idx = torch.arange(img.size(3) - 1, -1, -1).long()
        img_flip = img.index_select(3, inv_idx)
        return img_flip
    #확인
    def extract_feature(self, model, dataloaders):
        pbar = tqdm()
        if self.opt.linear_num <= 0:
            if self.opt.use_swin or self.opt.use_swinv2 or self.opt.use_dense or self.opt.use_convnext:
                self.opt.linear_num = 1024
            elif self.opt.use_efficient:
                self.opt.linear_num = 1792
            elif self.opt.use_NAS:
                self.opt.linear_num = 4032
            else:
                self.opt.linear_num = 2048

        for iter, data in enumerate(dataloaders):
            img, label = data
            n, c, h, w = img.size()
            pbar.update(n)
            ff = torch.FloatTensor(n, self.opt.linear_num).zero_().cuda()

            if self.opt.PCB:
                ff = torch.FloatTensor(n, 2048, 6).zero_().cuda()

            for i in range(2):
                if i == 1:
                    img = self.fliplr(img)
                input_img = Variable(img.cuda())
                for scale in self.ms:
                    if scale !=1:
                        input_img = nn.functional.interpolate(input_img, scale_factor=scale, mode='bicubic', align_corners=False)
                    outputs = model(input_img) 
                    ff += outputs

            if self.opt.PCB:
                fnorm = torch.norm(ff, p=2, dim=1, keepdim=True) * np.sqrt(6)
                ff = ff.div(fnorm.expand_as(ff))
                ff = ff.view(ff.size(0), -1)
            else:
                fnorm = torch.norm(ff, p=2, dim=1, keepdim=True)
                ff = ff.div(fnorm.expand_as(ff))

            if iter == 0:
                features = torch.FloatTensor(len(dataloaders.dataset), ff.shape[1])
            start = iter * self.opt.batchsize
            end = min((iter + 1) * self.opt.batchsize, len(dataloaders.dataset))
            features[start:end, :] = ff
        pbar.close()
        return features
    # 확인
    def get_id(self, img_path):
        camera_id, labels = [], []
        for path, v in img_path:
            filename = os.path.basename(path)
            label = filename[0:4]
            camera = filename.split('c')[1]
            if label[0:2] == '-1':
                labels.append(-1)
            else:
                labels.append(int(label))
            camera_id.append(int(camera[0]))
        return camera_id, labels
    #확인
    def load_model(self):
        if self.opt.use_dense:
            self.model_structure = ft_net_dense(self.opt.nclasses, stride=self.opt.stride, linear_num=self.opt.linear_num)
        elif self.opt.use_NAS:
            self.model_structure = ft_net_NAS(self.opt.nclasses, linear_num=self.opt.linear_num)
        elif self.opt.use_swin:
            self.model_structure = ft_net_swin(self.opt.nclasses, linear_num=self.opt.linear_num)
        elif self.opt.use_swinv2:
            self.model_structure = ft_net_swinv2(self.opt.nclasses, (self.h, self.w), linear_num=self.opt.linear_num)
        elif self.opt.use_convnext:
            self.model_structure = ft_net_convnext(self.opt.nclasses, linear_num=self.opt.linear_num)
        elif self.opt.use_efficient:
            self.model_structure = ft_net_efficient(self.opt.nclasses, linear_num=self.opt.linear_num)
        elif self.opt.use_hr:
            self.model_structure = ft_net_hr(self.opt.nclasses, linear_num=self.opt.linear_num)
        else:
            self.model_structure = ft_net(self.opt.nclasses, stride=self.opt.stride, ibn=self.opt.ibn, linear_num=self.opt.linear_num)

        if self.opt.PCB:
            self.model_structure = PCB(self.opt.nclasses)

        self.model = self.load_network(self.model_structure)
        
        if self.opt.PCB:
            self.model = PCB_test(self.model)
        else:
            self.model.classifier.classifier = nn.Sequential()

        self.model = self.model.eval()
        
        if torch.cuda.is_available():
            self.model = self.model.cuda()

        self.model = fuse_all_conv_bn(self.model)
        print(self.model)
        print('\n')

    def run(self,id):
        self.id=id
        gal_count=0
        result_gallery = scipy.io.loadmat('pytorch_result_gallery.mat') # 갤러리 특성 불러오기 
        #갤러리 특성 할당  
        gallery_feature = torch.FloatTensor(result_gallery['gallery_f'])
        gallery_cam = result_gallery['gallery_cam'][0]
        gallery_label = result_gallery['gallery_label'][0]
        k=id-min(self.existing_ids)
        while True:
            
            self.query_image_datasets = {x: datasets.ImageFolder( os.path.join(self.data_dir,x) ) for x in ['query']} # 쿼리데이터 셋
            result_query = scipy.io.loadmat('pytorch_result_query.mat') # 쿼리 특성 불러오기 반복
            query_feature = torch.FloatTensor(result_query['query_f'])  # 쿼리 특징  
            query_cam = result_query['query_cam'][0]
            query_label = result_query['query_label'][0]
            
            query_feature = query_feature.cuda()        #병렬 계산 cuda 사용
            gallery_feature = gallery_feature.cuda()
            
            index = self.sort_img(query_feature[k], query_label[k], query_cam[k], gallery_feature, gallery_label, gallery_cam) # k 번쨰 쿼리 이미지를 갤러리에서 검출
            
            # Visualize the rank result and determine the most likely ID
            query_path, _ = self.query_image_datasets['query'].imgs[k] # k 번째 이미지의 쿼리 경로
            query_label = query_label[k]
            print(query_path)
            print('Top 10 images are as follow:')
            id_counter = Counter()

            try:
                fig = plt.figure(figsize=(16, 4))
                ax = plt.subplot(1, 11, 1)
                ax.axis('off')
                self.imshow(query_path,f'query')
                for i in range(8):
                    ax = plt.subplot(1, 11, i+2)
                    ax.axis('off')
                    img_path, _ = self.image_datasets['gallery'].imgs[index[i]]             # 갤러리의 이미지의 경로
                    label = gallery_label[index[i]]
                    self.imshow(img_path)
                    id_counter[label] += 1
                    if label == query_label:
                        ax.set_title('%d_%s' % (i+1, 'True'), color='green', fontsize=12)
                    else:
                        ax.set_title('%d_%s' % (i+1, 'False'), color='red', fontsize=12)
                    #print(img_path)
                print(img_path)
            except RuntimeError:
                print('RuntimeError')
                
            # 가장 가능성 높은 ID와 매칭 수를 계산
            most_likely_id, count = id_counter.most_common(1)[0]
            if count >=5:
                if id == most_likely_id:
                    result_text=f"The query data matches the target ID: {id}."
                    result_color='blue'
                else:
                    result_text = f"Failed to matches a target ID: {id}\n But the most likely ID for the query image is: {most_likely_id} with {count} matches."
                    result_color='green'
                    
            else:
                result_text = "Failed to find a matching ID for the query image."
                result_color='red'

            # 텍스트를 이미지에 추가
            plt.suptitle(result_text, fontsize=25, color=result_color)

            # 이미지 파일로 저장
            fig.savefig(f"show0.png") # 저장
            
            
            #######################################################
            self.query_image_datasets = {x: datasets.ImageFolder( os.path.join(self.data_dir,x) ) for x in ['query']}
            self.gallery_path = self.image_datasets['gallery'].imgs           #  갤러리 이미지 경로 =
            self.query_path = self.query_image_datasets['query'].imgs           # 쿼리 이미지 경로

            self.gallery_cam,self.gallery_label = self.get_id(self.gallery_path)
            self.query_cam,self.query_label = self.get_id(self.query_path)
            since=time.time()
            # 새로운 데이터 로드
            print('start\n')
            with torch.no_grad():
                if gal_count<1:
                    print('갤러리 특성 추출\n')
                    gallery_feature = self.extract_feature(self.model,self.dataloaders['gallery'])
                    result_gallery = {'gallery_f':gallery_feature.numpy(),'gallery_label':self.gallery_label,'gallery_cam':self.gallery_cam}
                    scipy.io.savemat('pytorch_result_gallery.mat',result_gallery)
                    query_feature = self.extract_feature(self.model,self.dataloaders['query'])
                    result_query = {'query_f':query_feature.numpy(),'query_label':self.query_label,'query_cam':self.query_cam}
                    scipy.io.savemat('pytorch_result_query.mat',result_query)
                    print('갤러리 추출 끝\n')
                else:
                    self.load_data()
                gal_count+=1
                
                
                query_feature = self.extract_feature(self.model,self.dataloaders['query']) #쿼리 특징 추출
                result_query = {'query_f':query_feature.numpy(),'query_label':self.query_label,'query_cam':self.query_cam} #쿼리 특징 결과 
                scipy.io.savemat('pytorch_result_query.mat',result_query)      # 쿼리 결과 mat으로 저장
                
            time_elapsed = time.time() - since
            print('Training complete in {:.0f}m {:.2f}s'.format(
                        time_elapsed // 60, time_elapsed % 60)) 
            print('End\n')
    
    
    
    
if __name__ == '__main__':
    new_person=False
    id=1503
    
    event=threading.Event()
    
    detector = Detector(event)
    re_id=ReIDTester(event)
    
    if new_person:    
        detector.save_gallery2(new_person,id)
    else:
        detector.save_gallery2(new_person,id)
        re_id_thread=threading.Thread(target=re_id.run,args=(id,))
        detection_thread = threading.Thread(target=detector.detect_query)

        re_id_thread.start()
        time.sleep(15)
        detection_thread.start()
        
    ## 이미지 경로 바꿔줘야함
    ## 