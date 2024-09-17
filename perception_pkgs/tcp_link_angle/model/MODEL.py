import torch #print(torch.__version__) print(torch.version.cuda) cuda_available = torch.cuda.is_available()
import torch.nn as nn
import torch.nn.functional as F

device = torch.device('cuda')

weights = 1 / (torch.arange(1, 71)*1).float() 
weights = weights.to(device).unsqueeze(0).unsqueeze(2)

class myLoss(nn.modules.loss._Loss):
    def __init__(self):
        super(myLoss, self).__init__()

    def forward(self, prediction, target):
        batch_size, sequence_length, _ = prediction.size()

        prediction = prediction.cumsum(dim=1)
        target = target.cumsum(dim=1)

        abs_errors = torch.abs(prediction - target)
        squared_errors = abs_errors ** 2
        weighted_squared_errors = squared_errors * weights
        #weighted_squared_errors[:,:,1] *= 2 #lateral is much larger
        
        loss = torch.mean(weighted_squared_errors)

        return loss, abs_errors
    
########################################################################
class PositionalEncoding(nn.Module):
    def __init__(self, d_model, max_seq_len=30):
        super(PositionalEncoding, self).__init__()
        self.dropout = nn.Dropout(0.1)

        pe = torch.zeros(max_seq_len, d_model)
        position = torch.arange(0, max_seq_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(torch.arange(0, d_model, 2).float() * (-torch.log(torch.tensor(10000.0)) / d_model))
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        pe = pe.unsqueeze(0)
        self.register_buffer('pe', pe)

    def forward(self, x):
        x = x + self.pe[:, :x.size(1)]
        return self.dropout(x)



#2+2+16+8
class NN_Transformer_noEEG(nn.Module):
    def __init__(self, input_size=4, hidden_layer_size=32, output_size=2):
        super(NN_Transformer_noEEG, self).__init__()
        self.conv_eeg = nn.Conv1d(in_channels=32, out_channels=28, kernel_size=5, stride=5)
        self.conv_eeg2 = nn.Conv1d(in_channels=28, out_channels=24, kernel_size=3, stride=3)
        
        self.wave_time = nn.Linear(5, 30) #fixed
        self.smallwaves = nn.Linear(3*32, 8)
        
        #self.basic = nn.Linear(2, 16)


        self.embedding = nn.Linear(input_size, hidden_layer_size)
        self.positional_encoding = PositionalEncoding(hidden_layer_size, max_seq_len=20)  # Adjust max_seq_len
        self.transformer = nn.Transformer(
            d_model=hidden_layer_size,
            nhead=4,
            num_encoder_layers=2,
            num_decoder_layers=2,
            dim_feedforward=64,
            dropout=0.2,
            batch_first = True)
        self.output_layer = nn.Linear(hidden_layer_size, output_size)
        
        self.dropout_main = nn.Dropout(0.0)  # Adjust dropout rate as needed
        self.dropout_lstm = nn.Dropout(0.0)
        
    def forward(self, x_motion, x_eyes, x_eeg, x_waves):
        x = torch.cat([x_motion, x_eyes], axis = 2) #2+2+32+16
        
        x = self.dropout_main(x)
        
        x = self.embedding(x)
        x = self.positional_encoding(x)
        
        target_seq = torch.zeros(x.size(0), 1, x.size(2), device=x.device)
        transformer_output = self.transformer(x, target_seq)
        
        #transformer_output = self.transformer(x)
        output_sequence = self.output_layer(transformer_output)

        return output_sequence[:,-1,:] #.softmax(dim=1)
    
    
    
########################################################################

#2+2+16+8
class NN_Transformer_pos(nn.Module):
    def __init__(self, input_size=2, hidden_layer_size=32, output_size=2):
        super(NN_Transformer_pos, self).__init__()
        self.conv_eeg = nn.Conv1d(in_channels=32, out_channels=28, kernel_size=5, stride=5)
        self.conv_eeg2 = nn.Conv1d(in_channels=28, out_channels=24, kernel_size=3, stride=3)
        
        self.wave_time = nn.Linear(5, 30) #fixed
        self.smallwaves = nn.Linear(3*32, 8)
        
        #self.basic = nn.Linear(2, 16)


        self.embedding = nn.Linear(input_size, hidden_layer_size)
        self.positional_encoding = PositionalEncoding(hidden_layer_size, max_seq_len=20)  # Adjust max_seq_len
        self.transformer = nn.Transformer(
            d_model=hidden_layer_size,
            nhead=4,
            num_encoder_layers=2,
            num_decoder_layers=2,
            dim_feedforward=64,
            dropout=0.2,
            batch_first = True)
        self.output_layer = nn.Linear(hidden_layer_size, output_size)
        
        self.dropout_main = nn.Dropout(0.0)  # Adjust dropout rate as needed
        self.dropout_lstm = nn.Dropout(0.0)
        
    def forward(self, x_motion, x_eyes, x_eeg, x_waves):
        x = x_motion #2+2+32+16
        
        x = self.dropout_main(x)
        
        x = self.embedding(x)
        x = self.positional_encoding(x)
        
        target_seq = torch.zeros(x.size(0), 1, x.size(2), device=x.device)
        transformer_output = self.transformer(x, target_seq)
        
        #transformer_output = self.transformer(x)
        output_sequence = self.output_layer(transformer_output)

        return output_sequence[:,-1,:] #.softmax(dim=1)
    
    
    
########################################################################

#2+2+16+8
class NN_Transformer_eye(nn.Module):
    def __init__(self, input_size=2, hidden_layer_size=32, output_size=2):
        super(NN_Transformer_eye, self).__init__()
        self.conv_eeg = nn.Conv1d(in_channels=32, out_channels=28, kernel_size=5, stride=5)
        self.conv_eeg2 = nn.Conv1d(in_channels=28, out_channels=24, kernel_size=3, stride=3)
        
        self.wave_time = nn.Linear(5, 30) #fixed
        self.smallwaves = nn.Linear(3*32, 8)
        
        #self.basic = nn.Linear(2, 16)


        self.embedding = nn.Linear(input_size, hidden_layer_size)
        self.positional_encoding = PositionalEncoding(hidden_layer_size, max_seq_len=20)  # Adjust max_seq_len
        self.transformer = nn.Transformer(
            d_model=hidden_layer_size,
            nhead=4,
            num_encoder_layers=2,
            num_decoder_layers=2,
            dim_feedforward=64,
            dropout=0.2,
            batch_first = True)
        self.output_layer = nn.Linear(hidden_layer_size, output_size)
        
        self.dropout_main = nn.Dropout(0.0)  # Adjust dropout rate as needed
        self.dropout_lstm = nn.Dropout(0.0)
        
    def forward(self, x_motion, x_eyes, x_eeg, x_waves):
        x = x_eyes #2+2+32+16
        
        x = self.dropout_main(x)
        
        x = self.embedding(x)
        x = self.positional_encoding(x)
        
        target_seq = torch.zeros(x.size(0), 1, x.size(2), device=x.device)
        transformer_output = self.transformer(x, target_seq)
        
        #transformer_output = self.transformer(x)
        output_sequence = self.output_layer(transformer_output)

        return output_sequence[:,-1,:] #.softmax(dim=1)
    
########################################################################
    

#2+2+16+8
class NN_Transformer_full(nn.Module):
    def __init__(self, input_size=(4+24+8), hidden_layer_size=32, output_size=2):
        super(NN_Transformer_full, self).__init__()
        self.conv_eeg = nn.Conv1d(in_channels=32, out_channels=28, kernel_size=5, stride=5)
        self.conv_eeg2 = nn.Conv1d(in_channels=28, out_channels=24, kernel_size=5, stride=5)
        
        self.wave_time = nn.Linear(5, 20) #fixed
        self.smallwaves = nn.Linear(3*32, 8)
        
        #self.basic = nn.Linear(2, 16)


        self.embedding = nn.Linear(input_size, hidden_layer_size)
        self.positional_encoding = PositionalEncoding(hidden_layer_size, max_seq_len=20)  # Adjust max_seq_len
        self.transformer = nn.Transformer(
            d_model=hidden_layer_size,
            nhead=4,
            num_encoder_layers=2,
            num_decoder_layers=2,
            dim_feedforward=64,
            dropout=0.2,
            batch_first = True)
        self.output_layer = nn.Linear(hidden_layer_size, output_size)
        
        self.dropout_main = nn.Dropout(0.0)  # Adjust dropout rate as needed
        self.dropout_lstm = nn.Dropout(0.0)
        
    def forward(self, x_motion, x_eyes, x_eeg, x_waves):
        
        x_eeg = torch.transpose(x_eeg, 1, 2) #To batch, channels, time #batch,500,32
        eeg_conv = self.conv_eeg(x_eeg)
        #print(eeg_conv.shape)
        eeg_conv = self.conv_eeg2(eeg_conv)
        #print(eeg_conv.shape)
        eeg_conv = torch.transpose(eeg_conv, 1, 2)
        #print(eeg_conv.shape)
        
        x_waves = torch.flatten(x_waves, start_dim=2) #to batch, time, channels*3
        x_waves = torch.transpose(x_waves, 1, 2) #to batch, features, time
        waves_30 = F.sigmoid(self.wave_time(x_waves)) #changeing time dimension to 30
        waves_30 = torch.transpose(waves_30, 1, 2)
        waves_final = F.sigmoid(self.smallwaves(waves_30))
        
        
        x = torch.cat([x_motion, x_eyes, eeg_conv, waves_final], axis = 2) #2+2+32+16
        
        x = self.dropout_main(x)
        
        x = self.embedding(x)
        x = self.positional_encoding(x)
        
        target_seq = torch.zeros(x.size(0), 1, x.size(2), device=x.device)
        transformer_output = self.transformer(x, target_seq)
        
        #transformer_output = self.transformer(x)
        output_sequence = self.output_layer(transformer_output)

        return output_sequence[:,-1,:] #.softmax(dim=1)

########################################################################

class NN_LSTM(nn.Module):
    def __init__(self, input_size=4, hidden_layer_size=16, output_size=2): #48
        super(NN_LSTM, self).__init__()
        self.conv_eeg = nn.Conv1d(in_channels=32, out_channels=22, kernel_size=5, stride=5)
        self.conv_eeg2 = nn.Conv1d(in_channels=22, out_channels=12, kernel_size=3, stride=3)
        
        self.basic = nn.Linear(4, 28)
        self.wave_time = nn.Linear(5, 30) #fixed
        self.smallwaves = nn.Linear(3*32, 8)

        #self.lstm = nn.LSTM(input_size=input_size, hidden_size=hidden_layer_size, num_layers = 2, batch_first = True)
        #self.finallayer = nn.Linear(hidden_layer_size, output_size)
        
        self.lstm = nn.LSTM(input_size=input_size, hidden_size=hidden_layer_size, num_layers = 2, batch_first = True)
        self.finallayer = nn.Linear(hidden_layer_size, output_size)
        
        self.dropout_main = nn.Dropout(0.0)  # Adjust dropout rate as needed
        self.dropout_lstm = nn.Dropout(0.1)

        
    def forward(self, x_motion, x_eyes, x_eeg, x_waves):
        x = torch.cat([x_motion, x_eyes], axis = 2) #2+2+32+16
        
        x = self.dropout_main(x)
        
        
        lstm_output, hidden_state = self.lstm(x)
        lstm_output = self.dropout_lstm(lstm_output)

        output_sequence = self.finallayer(lstm_output)
    
        return output_sequence[:,-1,:] #.softmax(dim=1)


'''
from thop import profile


dummy_batch_size = 1
# Create dummy inputs
input1 = torch.randn(dummy_batch_size, 30, 2).cuda()
input2 = torch.randn(dummy_batch_size, 30, 2).cuda()
input3 = torch.randn(dummy_batch_size, 450, 32).cuda()
input4 = torch.randn(dummy_batch_size, 5, 3, 32).cuda()

net = NN_LSTM()
net = net.cuda()
flops, params = profile(net, inputs=(input1, input2, input3, input4))

print(f"FLOPs: {flops / 1e6} million")
print(f"Parameters: {params / 1e3} thousand")
'''

#Tranformer
#FLOPs: 0.8814 million
#Parameters: 15.242 thousand

#LSTM
#FLOPs: 1.73208 million
#Parameters: 24.356 thousand